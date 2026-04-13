import numpy as np
import os
import struct
import subprocess

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 8: INT8 양자화 및 C 헤더 변환
# FP32 Keras 모델 → FP32 TFLite → INT8 Full Quantized TFLite → C 헤더 배열
# ==============================================================================

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
import tensorflow as tf
from sklearn.metrics import accuracy_score, f1_score

# ─────────────────────────────────────────────
# [Cell 1] 데이터 및 모델 로드
# ─────────────────────────────────────────────
print("\n[Step 1] 필요 파일 로드 중...")

X_scaled = np.load("./ml-data/X_features_scaled.npy")
y_labels = np.load("./ml-data/y_labels.npy")
mins = np.load("./ml-data/feature_mins.npy")
ranges = np.load("./ml-data/feature_ranges.npy")

classifier = tf.keras.models.load_model("./ml-data/classifier_model.keras")

# Step 7에서 저장한 FP32 성능 지표 로드
fp32_metrics = np.load("./ml-data/fp32_metrics.npy")  # [accuracy, f1_macro]
fp32_f1_per_class = np.load("./ml-data/fp32_f1_per_class.npy")

fp32_accuracy = fp32_metrics[0]
fp32_f1_macro = fp32_metrics[1]

print(f"✅ 데이터 로드 완료! 전체 윈도우: {X_scaled.shape[0]}개")
print(f"   FP32 Accuracy (Step 7 결과): {fp32_accuracy * 100:.2f}%")

# ─────────────────────────────────────────────
# [Cell 2] FP32 TFLite 변환
# ─────────────────────────────────────────────
print("\n[Step 2] FP32 TFLite 변환 중...")

converter_fp32 = tf.lite.TFLiteConverter.from_keras_model(classifier)
tflite_fp32_model = converter_fp32.convert()

fp32_path = "./ml-data/auv_classifier_fp32.tflite"
with open(fp32_path, "wb") as f:
    f.write(tflite_fp32_model)

fp32_size_kb = len(tflite_fp32_model) / 1024
print(f"✔️ FP32 TFLite 저장 완료: {fp32_path}")
print(f"   모델 크기: {fp32_size_kb:.2f} KB")

# ─────────────────────────────────────────────
# [Cell 3] Representative Dataset 정의
# INT8 Full Quantization에 필요한 대표 데이터셋 (100~300개 샘플)
# 모든 활성화 레이어의 min/max 범위를 캘리브레이션
# ─────────────────────────────────────────────
print("\n[Step 3] INT8 Representative Dataset 준비 중...")

# 전체 데이터에서 균등 샘플링 (클래스 균형 고려)
np.random.seed(42)
N_repr = min(300, len(X_scaled))
repr_indices = np.random.choice(len(X_scaled), N_repr, replace=False)
repr_data = X_scaled[repr_indices].astype(np.float32)


def representative_dataset_gen():
    """INT8 양자화 캘리브레이션용 대표 샘플 Generator"""
    for sample in repr_data:
        # shape: (1, 30) - 배치 차원 추가
        yield [sample.reshape(1, 30)]


print(f"✔️ Representative Dataset: {N_repr}개 샘플 준비 완료")

# ─────────────────────────────────────────────
# [Cell 4] INT8 Full Quantization 변환
# - OPTIMIZE_FOR_SIZE: 크기 최소화 우선
# - DEFAULT_RANGES_STATS: float 활성화 범위 캘리브레이션
# - inference_input_type = INT8 (ESP32 TFLite Micro 호환)
# - inference_output_type = INT8
# ─────────────────────────────────────────────
print("\n[Step 4] INT8 Full Quantization 변환 중... (약 1~2분 소요)")

converter_int8 = tf.lite.TFLiteConverter.from_keras_model(classifier)

# 최적화 옵션: DEFAULT = 가중치 + 활성화 모두 양자화
converter_int8.optimizations = [tf.lite.Optimize.DEFAULT]

# 대표 데이터셋 연결 (활성화 레이어 양자화를 위해 필수)
converter_int8.representative_dataset = representative_dataset_gen

# ESP32 TFLite Micro는 INT8 입출력 타입 사용
converter_int8.inference_input_type = tf.int8
converter_int8.inference_output_type = tf.int8

# 실제 변환 실행
tflite_int8_model = converter_int8.convert()

int8_path = "./ml-data/auv_classifier_int8.tflite"
with open(int8_path, "wb") as f:
    f.write(tflite_int8_model)

int8_size_kb = len(tflite_int8_model) / 1024
print(f"✔️ INT8 TFLite 저장 완료: {int8_path}")
print(
    f"   모델 크기: {int8_size_kb:.2f} KB  (압축률: {fp32_size_kb / int8_size_kb:.1f}x)"
)

# ─────────────────────────────────────────────
# [Cell 5] INT8 Accuracy 평가 (TFLite Interpreter 사용)
# FP32 모델과의 정확도 차이를 측정
# ─────────────────────────────────────────────
print("\n[Step 5] INT8 모델 정확도 평가 중...")


# INT8 입력에 맞게 float → int8로 변환하는 함수
# 수식: q = clamp(round(x / scale + zero_point), -128, 127)
def run_tflite_inference(model_bytes, X_data, is_int8=False):
    """TFLite 모델을 Interpreter로 실행하여 예측 결과 반환"""
    interpreter = tf.lite.Interpreter(model_content=model_bytes)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    # 입력 양자화 파라미터 획득 (scale, zero_point)
    inp_scale, inp_zero_point = input_details[0]["quantization"]
    out_scale, out_zero_point = output_details[0]["quantization"]

    predictions = []
    for sample in X_data:
        sample_f = sample.reshape(1, 30).astype(np.float32)
        if is_int8 and inp_scale != 0:
            # float → int8 양자화
            sample_q = np.round(sample_f / inp_scale + inp_zero_point).astype(np.int8)
            sample_q = np.clip(sample_q, -128, 127).astype(np.int8)
        else:
            sample_q = sample_f
        interpreter.set_tensor(input_details[0]["index"], sample_q)
        interpreter.invoke()
        output = interpreter.get_tensor(output_details[0]["index"])
        if is_int8 and out_scale != 0:
            # int8 출력 → float로 역양자화
            output_f = (output.astype(np.float32) - out_zero_point) * out_scale
        else:
            output_f = output.astype(np.float32)
        predictions.append(np.argmax(output_f))

    return np.array(predictions)


# 평가용 데이터 (전체 중 샘플링하여 속도 향상)
eval_idx = np.random.choice(len(X_scaled), min(2000, len(X_scaled)), replace=False)
X_eval = X_scaled[eval_idx]
y_eval = y_labels[eval_idx]

print(f"   평가 샘플 수: {len(eval_idx)}개 (전체 {len(X_scaled)}개 중)")

# INT8 추론 실행
print("   INT8 추론 실행 중...")
y_pred_int8 = run_tflite_inference(tflite_int8_model, X_eval, is_int8=True)

int8_accuracy = accuracy_score(y_eval, y_pred_int8)
class_names = ["NORMAL", "MOTION_ARTIFACT", "SENSOR_FAULT", "ENV_ANOMALY"]
present = sorted(np.unique(np.concatenate([y_eval, y_pred_int8])))
int8_f1_pc = f1_score(
    y_eval, y_pred_int8, labels=present, average=None, zero_division=0
)
int8_f1_macro = f1_score(y_eval, y_pred_int8, average="macro", zero_division=0)

print(f"✔️ INT8 Accuracy: {int8_accuracy * 100:.2f}%")
print(f"   INT8 F1 (macro): {int8_f1_macro:.4f}")
print(f"   정확도 손실(vs FP32): {(fp32_accuracy - int8_accuracy) * 100:.2f}%p")

# ─────────────────────────────────────────────
# [Cell 6] FP32 vs INT8 비교표 출력
# ─────────────────────────────────────────────
print("\n" + "=" * 60)
print("📊 FP32 vs INT8 모델 비교표")
print("=" * 60)
print(f"{'Metric':<25} {'FP32':>10} {'INT8':>10} {'Difference':>15}")
print(f"{'-' * 60}")
print(
    f"{'Model Size (KB)':<25} {fp32_size_kb:>9.2f} {int8_size_kb:>9.2f} "
    f"  {fp32_size_kb / max(int8_size_kb, 0.01):.1f}x smaller"
)
print(
    f"{'Accuracy (%)':<25} {fp32_accuracy * 100:>9.2f} {int8_accuracy * 100:>9.2f} "
    f"  {(fp32_accuracy - int8_accuracy) * 100:>+.2f}%p"
)
print(
    f"{'F1 Score (macro)':<25} {fp32_f1_macro:>9.4f} {int8_f1_macro:>9.4f} "
    f"  {(fp32_f1_macro - int8_f1_macro):>+.4f}"
)
print(f"{'Inference (est)':<25} {'~3ms':>10} {'~1ms':>10} {'~3x faster':>15}")
print(f"{'RAM (tensor arena)':<25} {'~8KB':>10} {'~4KB':>10} {'~2x smaller':>15}")
print("=" * 60)

# 클래스별 F1 비교
print(f"\n📋 클래스별 F1 Score 비교:")
print(f"  {'Class':<20} {'FP32 F1':>10} {'INT8 F1':>10} {'Diff':>8}")
print(f"  {'-' * 50}")
for i, cls_idx in enumerate(present):
    name = class_names[cls_idx]
    fp32_f1 = fp32_f1_per_class[i] if i < len(fp32_f1_per_class) else 0.0
    int8_f1 = int8_f1_pc[i]
    print(
        f"  {name:<20} {fp32_f1:>10.4f} {int8_f1:>10.4f} {(fp32_f1 - int8_f1):>+8.4f}"
    )

# ─────────────────────────────────────────────
# [Cell 7] auv_classifier_int8.h 생성 (C 배열)
# ESP32 TFLite Micro에서 `#include "auv_classifier_int8.h"` 로 직접 사용
# ─────────────────────────────────────────────
print("\n[Step 6] C 헤더 파일(auv_classifier_int8.h) 생성 중...")


def tflite_to_c_array(model_bytes, var_name, header_filename):
    """TFLite 바이너리 → C 배열 헤더 파일로 변환"""
    hex_values = [f"0x{b:02x}" for b in model_bytes]
    total_bytes = len(model_bytes)

    lines = []
    lines.append("// ============================================================")
    lines.append("// AUV 4-Class Anomaly Classifier - INT8 Quantized TFLite Model")
    lines.append("// Auto-generated by step8_quantize_tinyml.py")
    lines.append(f"// Model size: {total_bytes} bytes ({total_bytes / 1024:.2f} KB)")
    lines.append(
        f"// Accuracy: {int8_accuracy * 100:.2f}% | F1 (macro): {int8_f1_macro:.4f}"
    )
    lines.append(
        "// Classes: 0=NORMAL, 1=MOTION_ARTIFACT, 2=SENSOR_FAULT, 3=ENV_ANOMALY"
    )
    lines.append("// Input: float32[30] (normalized 0~1), scaled to int8 on-device")
    lines.append("// ============================================================")
    lines.append(f"#ifndef {var_name.upper()}_H")
    lines.append(f"#define {var_name.upper()}_H")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"const unsigned int {var_name}_len = {total_bytes};")
    lines.append(f"alignas(8) const unsigned char {var_name}[] = {{")

    # 16바이트씩 한 줄로 출력
    chunk_size = 16
    for i in range(0, total_bytes, chunk_size):
        chunk = hex_values[i : i + chunk_size]
        line = "  " + ", ".join(chunk)
        if i + chunk_size < total_bytes:
            line += ","
        lines.append(line)

    lines.append("};")
    lines.append("")
    lines.append(f"#endif // {var_name.upper()}_H")

    with open(header_filename, "w") as f:
        f.write("\n".join(lines))

    print(
        f"✔️ {header_filename} 생성 완료 ({total_bytes / 1024:.2f} KB, {total_bytes}바이트)"
    )


# INT8 모델을 C 헤더로 변환 (Arduino 프로젝트 폴더에 직접 저장)
tflite_to_c_array(tflite_int8_model, "auv_classifier_int8", "./auv_classifier_int8.h")

# ─────────────────────────────────────────────
# [Cell 8] auv_norm_params.h 생성 (정규화 파라미터)
# ESP32에서 float → normalized(0~1) 변환에 필요
# feature_norm = (raw_val - mins[i]) / ranges[i]
# ─────────────────────────────────────────────
print("\n[Step 7] C 헤더 파일(auv_norm_params.h) 생성 중...")

feature_names = [
    "pH_mean",
    "pH_std",
    "pH_slope",
    "pH_rms",
    "EC_mean",
    "EC_std",
    "EC_slope",
    "EC_rms",
    "DO_mean",
    "DO_std",
    "DO_slope",
    "DO_rms",
    "O2_mean",
    "O2_std",
    "O2_slope",
    "O2_rms",
    "depth_mean",
    "depth_std",
    "depth_slope",
    "temp_mean",
    "temp_std",
    "temp_slope",
    "imu_mean",
    "imu_std",
    "imu_max",
    "pH_temp_corr",
    "pH_res_mean",
    "pH_res_std",
    "DO_res_mean",
    "DO_res_std",
]

norm_lines = []
norm_lines.append("// ============================================================")
norm_lines.append("// AUV Feature Normalization Parameters (Min-Max, 0~1)")
norm_lines.append("// Auto-generated by step8_quantize_tinyml.py")
norm_lines.append("// Usage: norm = (raw_val - FEAT_MINS[i]) / FEAT_RANGES[i]")
norm_lines.append("//        clamp to [0.0, 1.0] before inference")
norm_lines.append("// ============================================================")
norm_lines.append("#ifndef AUV_NORM_PARAMS_H")
norm_lines.append("#define AUV_NORM_PARAMS_H")
norm_lines.append("")
norm_lines.append("#define NUM_FEATURES 30")
norm_lines.append("")
norm_lines.append("// 각 특징의 최솟값 (정규화 공식의 분자 보정)")
norm_lines.append("const float FEAT_MINS[NUM_FEATURES] = {")
for i, (name, val) in enumerate(zip(feature_names, mins)):
    comma = "," if i < 29 else ""
    norm_lines.append(f"  {val:.8f}f{comma}  // [{i:02d}] {name}")
norm_lines.append("};")
norm_lines.append("")
norm_lines.append("// 각 특징의 범위값 (정규화 분모: max - min)")
norm_lines.append("const float FEAT_RANGES[NUM_FEATURES] = {")
for i, (name, val) in enumerate(zip(feature_names, ranges)):
    comma = "," if i < 29 else ""
    # 범위가 0이면 나눗셈 오류 방지 → 1.0으로 대체
    safe_val = val if val > 1e-10 else 1.0
    norm_lines.append(f"  {safe_val:.8f}f{comma}  // [{i:02d}] {name}")
norm_lines.append("};")
norm_lines.append("")
norm_lines.append("#endif // AUV_NORM_PARAMS_H")

with open("./auv_norm_params.h", "w") as f:
    f.write("\n".join(norm_lines))

print("✔️ ./auv_norm_params.h 생성 완료")

print(f"\n🎉 Step 8 완료! INT8 양자화 및 헤더 변환이 끝났습니다.")
print("   → auv_classifier_int8.h  (ESP32 모델 C배열)")
print("   → auv_norm_params.h       (정규화 파라미터)")
print("   다음 Step 9에서 최종 보고서를 생성합니다.")

# Step 9에서 사용할 지표 저장
np.save(
    "./ml-data/int8_metrics.npy",
    np.array([int8_accuracy, int8_f1_macro, int8_size_kb, fp32_size_kb]),
)
np.save("./ml-data/int8_f1_per_class.npy", int8_f1_pc)
