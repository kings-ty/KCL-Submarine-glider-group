#ifndef TFLITE_INFERENCE_H
#define TFLITE_INFERENCE_H

#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

// ── 학습된 모델 & 정규화 파라미터 ──
// train_auv_model.py 실행 후 생성되는 파일들
#include "auv_classifier_int8.h"   // INT8 quantized model C array
#include "auv_norm_params.h"       // feature_mins[], feature_ranges[]

// ============================================================
// 설정
// ============================================================
#define TENSOR_ARENA_SIZE  8192   // 8KB — 이 모델에 충분 (부족하면 늘리기)
#define TFLITE_NUM_INPUTS  30     // 30개 특징
#define TFLITE_NUM_OUTPUTS 4      // 4개 클래스

// ============================================================
// TFLite Micro 전역 변수
// ============================================================
static const tflite::Model* tflite_model = nullptr;
static tflite::MicroInterpreter* tflite_interpreter = nullptr;
static TfLiteTensor* tflite_input = nullptr;
static TfLiteTensor* tflite_output = nullptr;

// Tensor Arena — 모델 실행에 필요한 작업 메모리
alignas(16) static uint8_t tensor_arena[TENSOR_ARENA_SIZE];

// 성능 측정용
struct InferenceMetrics {
  float    latency_us;        // 마지막 inference 소요시간 (마이크로초)
  float    avg_latency_us;    // 평균 latency
  uint32_t total_inferences;  // 총 inference 횟수
  float    max_latency_us;    // 최대 latency
  float    min_latency_us;    // 최소 latency
  uint32_t arena_used_bytes;  // 실제 사용된 Tensor Arena 크기
};

static InferenceMetrics g_metrics = {0, 0, 0, 0, 999999.0f, 0};

// ============================================================
// 초기화
// ============================================================
bool tflite_init() {
  Serial.println("[TFLite] Initializing...");

  // 1. 모델 로드
  tflite_model = tflite::GetModel(auv_cls_model);
  if (tflite_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.printf("[TFLite] ERROR: Model schema version %d != expected %d\n",
                  tflite_model->version(), TFLITE_SCHEMA_VERSION);
    return false;
  }

  // 2. Op Resolver (모든 연산 포함 — 나중에 필요한 것만으로 최적화 가능)
  static tflite::AllOpsResolver resolver;

  // 3. Interpreter 생성
  static tflite::MicroInterpreter static_interpreter(
      tflite_model, resolver, tensor_arena, TENSOR_ARENA_SIZE);
  tflite_interpreter = &static_interpreter;

  // 4. 텐서 할당
  TfLiteStatus allocate_status = tflite_interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("[TFLite] ERROR: AllocateTensors() failed!");
    return false;
  }

  // 5. 입출력 텐서 참조
  tflite_input  = tflite_interpreter->input(0);
  tflite_output = tflite_interpreter->output(0);

  // 6. 메모리 사용량 기록
  g_metrics.arena_used_bytes = tflite_interpreter->arena_used_bytes();

  // 7. 검증 출력
  Serial.println("[TFLite] ✅ Initialized successfully!");
  Serial.printf("  Model size:      %d bytes (%.1f KB)\n", auv_cls_model_len, auv_cls_model_len / 1024.0f);
  Serial.printf("  Arena allocated: %d bytes (%.1f KB)\n", TENSOR_ARENA_SIZE, TENSOR_ARENA_SIZE / 1024.0f);
  Serial.printf("  Arena used:      %lu bytes (%.1f KB)\n", g_metrics.arena_used_bytes, g_metrics.arena_used_bytes / 1024.0f);
  Serial.printf("  Input:  type=%d dim=%d\n", tflite_input->type, tflite_input->dims->data[1]);
  Serial.printf("  Output: type=%d dim=%d\n", tflite_output->type, tflite_output->dims->data[1]);

  // INT8 양자화 파라미터 확인
  if (tflite_input->type == kTfLiteInt8) {
    Serial.printf("  Input  quantization: scale=%.6f, zero_point=%d\n",
                  tflite_input->params.scale, tflite_input->params.zero_point);
    Serial.printf("  Output quantization: scale=%.6f, zero_point=%d\n",
                  tflite_output->params.scale, tflite_output->params.zero_point);
    Serial.println("  ✅ INT8 Quantization confirmed!");
  } else {
    Serial.println("  ⚠️  Model is FP32 (not quantized)");
  }

  return true;
}


// ============================================================
// 정규화 — Python과 동일한 Min-Max 적용
// ============================================================
void normalize_features(float* raw_features, float* normalized, int num_features) {
  for (int i = 0; i < num_features; i++) {
    if (feature_ranges[i] > 1e-10f) {
      normalized[i] = (raw_features[i] - feature_mins[i]) / feature_ranges[i];
    } else {
      normalized[i] = 0.0f;
    }
    // Clamp to [0, 1]
    if (normalized[i] < 0.0f) normalized[i] = 0.0f;
    if (normalized[i] > 1.0f) normalized[i] = 1.0f;
  }
}


// ============================================================
// Inference 실행 — 핵심 함수!
// ============================================================
// 
// 반환값: 0=NORMAL, 1=MOTION_ART, 2=SENSOR_FAULT, 3=ENV_ANOMALY
// confidence: 해당 클래스의 확신도 (0.0 ~ 1.0)
//
int tflite_classify(float* raw_features, float* confidence) {
  if (tflite_interpreter == nullptr) {
    *confidence = 0.0f;
    return 0;
  }

  // ── 1. 정규화 ──
  float normalized[TFLITE_NUM_INPUTS];
  normalize_features(raw_features, normalized, TFLITE_NUM_INPUTS);

  // ── 2. 입력 텐서에 데이터 쓰기 ──
  if (tflite_input->type == kTfLiteInt8) {
    // INT8: float → int8 양자화
    float scale = tflite_input->params.scale;
    int   zp    = tflite_input->params.zero_point;
    int8_t* input_data = tflite_input->data.int8;

    for (int i = 0; i < TFLITE_NUM_INPUTS; i++) {
      int32_t quantized = (int32_t)(normalized[i] / scale + zp);
      // Clamp to int8 range
      if (quantized < -128) quantized = -128;
      if (quantized > 127)  quantized = 127;
      input_data[i] = (int8_t)quantized;
    }
  } else {
    // FP32: 직접 복사
    float* input_data = tflite_input->data.f;
    memcpy(input_data, normalized, sizeof(float) * TFLITE_NUM_INPUTS);
  }

  // ── 3. Inference 실행 + 시간 측정 ──
  uint32_t start_us = micros();
  TfLiteStatus invoke_status = tflite_interpreter->Invoke();
  uint32_t end_us = micros();

  if (invoke_status != kTfLiteOk) {
    Serial.println("[TFLite] ERROR: Invoke() failed!");
    *confidence = 0.0f;
    return 0;
  }

  // ── 4. Latency 기록 ──
  float latency = (float)(end_us - start_us);
  g_metrics.latency_us = latency;
  g_metrics.total_inferences++;

  // Running average 계산
  g_metrics.avg_latency_us =
      g_metrics.avg_latency_us +
      (latency - g_metrics.avg_latency_us) / g_metrics.total_inferences;

  if (latency > g_metrics.max_latency_us) g_metrics.max_latency_us = latency;
  if (latency < g_metrics.min_latency_us) g_metrics.min_latency_us = latency;

  // ── 5. 출력 텐서 읽기 ──
  float output_probs[TFLITE_NUM_OUTPUTS];

  if (tflite_output->type == kTfLiteInt8) {
    // INT8 → float 역양자화
    float scale = tflite_output->params.scale;
    int   zp    = tflite_output->params.zero_point;
    int8_t* output_data = tflite_output->data.int8;

    for (int i = 0; i < TFLITE_NUM_OUTPUTS; i++) {
      output_probs[i] = (output_data[i] - zp) * scale;
    }
  } else {
    memcpy(output_probs, tflite_output->data.f, sizeof(float) * TFLITE_NUM_OUTPUTS);
  }

  // ── 6. argmax로 클래스 결정 ──
  int best_class = 0;
  float best_prob = output_probs[0];
  for (int i = 1; i < TFLITE_NUM_OUTPUTS; i++) {
    if (output_probs[i] > best_prob) {
      best_prob = output_probs[i];
      best_class = i;
    }
  }

  *confidence = best_prob;
  return best_class;
}


// ============================================================
// 성능 보고서 출력 — 시리얼 모니터에서 확인
// ============================================================
void tflite_print_metrics() {
  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║   TFLite Micro Performance Report        ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.printf( "║  Model size:        %6d bytes (%4.1f KB)║\n", auv_cls_model_len, auv_cls_model_len / 1024.0f);
  Serial.printf( "║  Tensor Arena:      %6d bytes (%4.1f KB)║\n", g_metrics.arena_used_bytes, g_metrics.arena_used_bytes / 1024.0f);
  Serial.printf( "║  Total inferences:  %6lu              ║\n", g_metrics.total_inferences);
  Serial.printf( "║  Latency (last):    %6.1f µs (%4.2f ms) ║\n", g_metrics.latency_us, g_metrics.latency_us / 1000.0f);
  Serial.printf( "║  Latency (avg):     %6.1f µs (%4.2f ms) ║\n", g_metrics.avg_latency_us, g_metrics.avg_latency_us / 1000.0f);
  Serial.printf( "║  Latency (min):     %6.1f µs             ║\n", g_metrics.min_latency_us);
  Serial.printf( "║  Latency (max):     %6.1f µs             ║\n", g_metrics.max_latency_us);
  Serial.printf( "║  Quantization:      %s                   ║\n",
                 tflite_input->type == kTfLiteInt8 ? "INT8  " : "FP32  ");
  Serial.println("╚══════════════════════════════════════════╝\n");
}


// ============================================================
// 메트릭 getter (LoRa 패킷에 포함용)
// ============================================================
float tflite_get_avg_latency_ms() {
  return g_metrics.avg_latency_us / 1000.0f;
}

uint32_t tflite_get_arena_used() {
  return g_metrics.arena_used_bytes;
}

uint32_t tflite_get_total_inferences() {
  return g_metrics.total_inferences;
}


#endif // TFLITE_INFERENCE_H
