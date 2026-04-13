// // 최소 TFLite 테스트 — Chirale 라이브러리 사용
// #include "Arduino.h"
// #include <Chirale_TensorFlowLite.h>
// #include "tensorflow/lite/micro/all_ops_resolver.h"
// #include "tensorflow/lite/micro/micro_interpreter.h"
// #include "tensorflow/lite/schema/schema_generated.h"
// #include "auv_classifier_int8.h"

// void setup() {
//   Serial.begin(115200);
//   delay(2000);
//   Serial.println("\n====== TFLite MINIMAL TEST (Chirale) ======");

//   // 1. 모델 파싱
//   const tflite::Model* model = tflite::GetModel(auv_classifier_int8);
//   if (!model) {
//     Serial.println("[FAIL] GetModel returned null");
//     while(1);
//   }
//   Serial.printf("[OK] Model parsed, size=%d bytes\n", auv_classifier_int8_len);

//   // 2. Op resolver
//   static tflite::AllOpsResolver resolver;
//   Serial.println("[OK] AllOpsResolver created");

//   // 3. Arena 할당 (힙)
//   constexpr int kArenaSize = 16 * 1024;
//   uint8_t* arena = (uint8_t*)malloc(kArenaSize);
//   if (!arena) {
//     Serial.println("[FAIL] malloc failed");
//     while(1);
//   }
//   Serial.printf("[OK] Arena allocated: %d bytes\n", kArenaSize);

//   // 4. 인터프리터 생성
//   Serial.println("[...] Creating interpreter...");
//   static tflite::MicroInterpreter interp(model, resolver, arena, kArenaSize);
//   Serial.println("[OK] Interpreter created");

//   // 5. 텐서 할당
//   TfLiteStatus status = interp.AllocateTensors();
//   if (status != kTfLiteOk) {
//     Serial.println("[FAIL] AllocateTensors failed!");
//     while(1);
//   }
//   Serial.printf("[OK] Tensors allocated, used=%d bytes\n", interp.arena_used_bytes());

//   // 6. 입출력 확인
//   TfLiteTensor* input = interp.input(0);
//   TfLiteTensor* output = interp.output(0);
//   Serial.printf("[OK] Input:  dims=%d, type=%d\n", input->dims->data[1], input->type);
//   Serial.printf("[OK] Output: dims=%d, type=%d\n", output->dims->data[1], output->type);

//   // 7. 더미 추론
//   for (int i = 0; i < input->dims->data[1]; i++) {
//     input->data.int8[i] = 0;
//   }
//   status = interp.Invoke();
//   if (status != kTfLiteOk) {
//     Serial.println("[FAIL] Invoke failed!");
//     while(1);
//   }
//   Serial.println("[OK] Inference completed!");

//   // 8. 결과 출력
//   int out_size = output->dims->data[1];
//   Serial.print("Output: [");
//   for (int i = 0; i < out_size; i++) {
//     Serial.print(output->data.int8[i]);
//     if (i < out_size - 1) Serial.print(", ");
//   }
//   Serial.println("]");

//   Serial.println("\n====== ALL TESTS PASSED ======");
// }

// void loop() {
//   delay(5000);
//   Serial.println("[ALIVE] running...");
// }
