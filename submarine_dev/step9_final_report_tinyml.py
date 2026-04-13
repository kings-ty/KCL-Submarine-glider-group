import numpy as np
import os
import datetime
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 9: 최종 보고서 생성
# 논문에 바로 활용 가능한 성능 비교표, Confusion Matrix 고품질 저장,
# 학습 전 과정 요약 텍스트 보고서 출력
# ==============================================================================

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
from sklearn.metrics import confusion_matrix, f1_score, accuracy_score, classification_report

# ─────────────────────────────────────────────
# [Cell 1] 저장된 지표 로드
# ─────────────────────────────────────────────
print("\n[Step 1] Step 7/8 결과 파일 로드 중...")

X_scaled  = np.load('./ml-data/X_features_scaled.npy')
y_labels  = np.load('./ml-data/y_labels.npy')

fp32_metrics      = np.load('./ml-data/fp32_metrics.npy')   # [acc, f1_macro]
fp32_f1_per_class = np.load('./ml-data/fp32_f1_per_class.npy')
int8_metrics      = np.load('./ml-data/int8_metrics.npy')   # [acc, f1_macro, int8_kb, fp32_kb]
int8_f1_per_class = np.load('./ml-data/int8_f1_per_class.npy')

fp32_acc,  fp32_f1  = fp32_metrics[0], fp32_metrics[1]
int8_acc,  int8_f1  = int8_metrics[0], int8_metrics[1]
int8_kb,   fp32_kb  = int8_metrics[2], int8_metrics[3]

class_names = ['NORMAL', 'MOTION_ARTIFACT', 'SENSOR_FAULT', 'ENV_ANOMALY']
print("✅ 지표 로드 완료")

# ─────────────────────────────────────────────
# [Cell 2] 고품질 Confusion Matrix 재생성 및 저장
# (Step 7 그림보다 더 완성도 높은 버전)
# ─────────────────────────────────────────────
print("\n[Step 2] 논문용 고품질 Confusion Matrix 생성 중...")

# INT8 모델로 전체 데이터 재추론
int8_bytes = open('./ml-data/auv_classifier_int8.tflite', 'rb').read()

interpreter = tf.lite.Interpreter(model_content=int8_bytes)
interpreter.allocate_tensors()
input_details  = interpreter.get_input_details()
output_details = interpreter.get_output_details()
inp_scale, inp_zp = input_details[0]['quantization']
out_scale, out_zp = output_details[0]['quantization']

# 샘플링 (전체 대신 2000개로 속도 향상)
np.random.seed(0)
eval_idx  = np.random.choice(len(X_scaled), min(2000, len(X_scaled)), replace=False)
X_eval    = X_scaled[eval_idx]
y_eval    = y_labels[eval_idx]

y_pred_int8 = []
for sample in X_eval:
    sf = sample.reshape(1, 30).astype(np.float32)
    if inp_scale != 0:
        sq = np.clip(np.round(sf / inp_scale + inp_zp), -128, 127).astype(np.int8)
    else:
        sq = sf.astype(np.int8)
    interpreter.set_tensor(input_details[0]['index'], sq)
    interpreter.invoke()
    out = interpreter.get_tensor(output_details[0]['index'])
    if out_scale != 0:
        out_f = (out.astype(np.float32) - out_zp) * out_scale
    else:
        out_f = out.astype(np.float32)
    y_pred_int8.append(np.argmax(out_f))

y_pred_int8 = np.array(y_pred_int8)

present = sorted(np.unique(np.concatenate([y_eval, y_pred_int8])))
target_names = [class_names[i] for i in present]

cm = confusion_matrix(y_eval, y_pred_int8, labels=present)

# 정규화 Confusion Matrix (열 방향 비율)
cm_norm = cm.astype('float') / cm.sum(axis=1, keepdims=True)

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# 절대값 CM
sns.heatmap(cm, annot=True, fmt='d', cmap='YlOrRd',
            xticklabels=target_names, yticklabels=target_names,
            linewidths=0.5, linecolor='gray',
            annot_kws={'size': 12}, ax=axes[0])
axes[0].set_title('Confusion Matrix (Count)', fontsize=13, fontweight='bold')
axes[0].set_xlabel('Predicted Label', fontsize=11)
axes[0].set_ylabel('True Label', fontsize=11)
axes[0].tick_params(axis='x', rotation=30)

# 정규화 CM (비율)
sns.heatmap(cm_norm, annot=True, fmt='.2f', cmap='Blues',
            xticklabels=target_names, yticklabels=target_names,
            linewidths=0.5, linecolor='gray',
            annot_kws={'size': 12}, vmin=0, vmax=1, ax=axes[1])
axes[1].set_title('Confusion Matrix (Normalized)', fontsize=13, fontweight='bold')
axes[1].set_xlabel('Predicted Label', fontsize=11)
axes[1].set_ylabel('True Label', fontsize=11)
axes[1].tick_params(axis='x', rotation=30)

fig.suptitle('AUV 4-Class Anomaly Classifier (INT8 Quantized)\nInWaterSense Dataset',
             fontsize=14, fontweight='bold', y=1.02)
plt.tight_layout()

plt.savefig('./confusion_matrix.png', dpi=200, bbox_inches='tight')
print("✔️ confusion_matrix.png 저장 완료 (200dpi, 논문 품질)")

plt.close('all')

# ─────────────────────────────────────────────
# [Cell 3] 논문용 성능 비교표 출력 (콘솔)
# ─────────────────────────────────────────────
print("\n" + "="*65)
print("📄 논문용 Final Performance Table")
print("="*65)

# 크기 압축률 계산
size_ratio = fp32_kb / max(int8_kb, 0.01)
acc_diff   = (fp32_acc - int8_acc) * 100  # %p 차이

print(f"\n{'Metric':<28} {'FP32':>10} {'INT8':>10} {'Difference':>12}")
print(f"{'-'*62}")
print(f"{'Model size':<28} {fp32_kb:>7.2f} KB {int8_kb:>7.2f} KB  {size_ratio:.1f}x smaller")
print(f"{'Accuracy (%)':<28} {fp32_acc*100:>9.2f}% {int8_acc*100:>9.2f}%  {acc_diff:>+.2f}%p")
print(f"{'F1 Score (macro)':<28} {fp32_f1:>10.4f} {int8_f1:>10.4f}  {(fp32_f1-int8_f1):>+.4f}")
print(f"{'Inference (estimated)':<28} {'~3 ms':>10} {'~1 ms':>10}  {'~3x faster':>12}")
print(f"{'RAM (tensor arena)':<28} {'~8 KB':>10} {'~4 KB':>10}  {'~2x smaller':>12}")
print(f"{'-'*62}")

print(f"\n  * Estimated for ESP32 @ 240MHz with TFLite Micro")
print(f"  * Dataset: InWaterSense (Ahmedi 2021, Mendeley)")
print(f"  * Window: {64} samples, Stride: {16}, Features: 30")

# 클래스 별 F1 비교
print(f"\n{'Class-wise F1 Score:'}")
print(f"  {'Class':<22} {'FP32':>8} {'INT8':>8} {'Delta':>8}")
print(f"  {'-'*50}")
for i, cls_idx in enumerate(present):
    name = class_names[cls_idx]
    f_fp32 = fp32_f1_per_class[i] if i < len(fp32_f1_per_class) else 0.0
    f_int8 = int8_f1_per_class[i] if i < len(int8_f1_per_class) else 0.0
    print(f"  {name:<22} {f_fp32:>8.4f} {f_int8:>8.4f} {(f_fp32-f_int8):>+8.4f}")

print("="*65)

# ─────────────────────────────────────────────
# [Cell 4] training_report.txt 저장
# ─────────────────────────────────────────────
print("\n[Step 3] training_report.txt 저장 중...")

# 클래스별 샘플 수 계산
label_counts = {class_names[i]: int((y_labels == i).sum()) for i in range(4)}
total_windows = len(y_labels)

report_lines = [
    "=" * 65,
    "AUV Anomaly Detection System - Final Training Report",
    "InWaterSense Dataset (Ahmedi 2021, Mendeley Data)",
    f"Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
    "=" * 65,
    "",
    "[ DATASET SUMMARY ]",
    f"  Dataset       : InWaterSense",
    f"  Channels      : pH, EC, DO (O2=0, depth=0, IMU=0)",
    f"  Window Size   : 64 samples (10-min data, 10-min interval)",
    f"  Window Stride : 16",
    f"  Feature Dim   : 30 per window",
    f"  Total Windows : {total_windows:,}",
    "",
    "[ LABEL DISTRIBUTION ]",
]
for name, cnt in label_counts.items():
    report_lines.append(f"  {name:<20}: {cnt:>8,} ({100.0*cnt/total_windows:.1f}%)")

report_lines += [
    "",
    "[ AUTOENCODER (Step 5) ]",
    "  Architecture : 30 → 16(relu) → 8(relu) → 16(relu) → 30(sigmoid)",
    "  Loss         : MSE",
    "  Optimizer    : Adam (lr=0.001)",
    "  Epochs       : 100 (with EarlyStopping)",
    "  Batch Size   : 32",
    "  Val Split    : 15%",
    "",
    "[ CLASSIFIER (Step 7) ]",
    "  Architecture : 30 → 32(relu, drop 0.2) → 16(relu, drop 0.1) → 4(softmax)",
    "  Loss         : Categorical Crossentropy",
    "  Optimizer    : Adam (lr=0.001)",
    "  Epochs       : 80 (with EarlyStopping + ReduceLROnPlateau)",
    "  Batch Size   : 32",
    "  Val Split    : 15%",
    "  Train/Test   : 80% / 20% (stratified)",
    "",
    "[ QUANTIZATION (Step 8) ]",
    "  Method       : INT8 Full Quantization (tf.lite.Optimize.DEFAULT)",
    "  Rep. Dataset : 300 stratified samples",
    "  Input Type   : INT8 (ESP32 TFLite Micro compatible)",
    "  Output Type  : INT8",
    "",
    "[ PERFORMANCE COMPARISON ]",
    f"  {'Metric':<28} {'FP32':>10} {'INT8':>10} {'Δ':>8}",
    f"  {'-'*58}",
    f"  {'Model Size (KB)':<28} {fp32_kb:>9.2f}  {int8_kb:>9.2f}  {size_ratio:>+.1f}x",
    f"  {'Accuracy (%)':<28} {fp32_acc*100:>9.2f}  {int8_acc*100:>9.2f}  {acc_diff:>+.2f}%p",
    f"  {'F1 Score (macro)':<28} {fp32_f1:>10.4f}  {int8_f1:>10.4f}  {(fp32_f1-int8_f1):>+.6f}",
    f"  {'Inference (est, ESP32)':<28} {'~3 ms':>10}  {'~1 ms':>10}  {'~3x faster':>10}",
    f"  {'RAM tensor arena (est)':<28} {'~8 KB':>10}  {'~4 KB':>10}  {'~2x smaller':>10}",
    "",
    "[ CLASS-WISE F1 SCORE ]",
    f"  {'Class':<22} {'FP32 F1':>9} {'INT8 F1':>9}",
    f"  {'-'*42}",
]
for i, cls_idx in enumerate(present):
    name = class_names[cls_idx]
    f_fp32 = fp32_f1_per_class[i] if i < len(fp32_f1_per_class) else 0.0
    f_int8 = int8_f1_per_class[i] if i < len(int8_f1_per_class) else 0.0
    report_lines.append(f"  {name:<22} {f_fp32:>9.4f} {f_int8:>9.4f}")

report_lines += [
    "",
    "[ OUTPUT FILES ]",
    "  ./auv_classifier_int8.h   : C array (ESP32 TFLite Micro)",
    "  ./auv_norm_params.h       : Normalization parameters (FEAT_MINS, FEAT_RANGES)",
    "  ./confusion_matrix.png    : Confusion matrix (200dpi)",
    "  ./training_report.txt     : This report",
    "",
    "[ NOTES ]",
    "  - O2, depth, IMU channels are zeroed (InWaterSense = river surface)",
    "  - Activate when deploying on actual AUV with full sensor suite",
    "  - MOTION_ARTIFACT class likely empty (IMU=0 in this dataset)",
    "  - Retrain with AUV-collected data for production deployment",
    "=" * 65,
]

report_text = '\n'.join(report_lines)

with open('./training_report.txt', 'w') as f:
    f.write(report_text)

# 콘솔 출력
print(report_text)

# ─────────────────────────────────────────────
# [Cell 5] 전체 Pipeline 파일 목록 요약 출력
# ─────────────────────────────────────────────
print("\n" + "="*65)
print("🗂️  전체 파이프라인 출력 파일 목록")
print("="*65)

output_files = [
    ("ml-data/step2_preprocessed_data.csv",   "Step 2 전처리 완료 데이터"),
    ("ml-data/X_features.npy",                "Step 3 원시 특징 배열 (N×30)"),
    ("ml-data/X_features_scaled.npy",         "Step 4 정규화 특징 배열"),
    ("ml-data/feature_mins.npy",              "Step 4 정규화 mins"),
    ("ml-data/feature_ranges.npy",            "Step 4 정규화 ranges"),
    ("ml-data/autoencoder_model.keras",        "Step 5 Autoencoder 모델"),
    ("ml-data/y_labels.npy",                  "Step 6 자동 라벨 배열"),
    ("ml-data/recon_errors.npy",              "Step 6 복원 오차 배열"),
    ("ml-data/classifier_model.keras",         "Step 7 Classifier 모델"),
    ("ml-data/auv_classifier_fp32.tflite",    "Step 8 FP32 TFLite"),
    ("ml-data/auv_classifier_int8.tflite",    "Step 8 INT8 TFLite"),
    ("auv_classifier_int8.h",                 "Step 8 ESP32 C배열 헤더 ★"),
    ("auv_norm_params.h",                     "Step 8 정규화 파라미터 헤더 ★"),
    ("confusion_matrix.png",                  "Step 9 논문용 Confusion Matrix"),
    ("training_report.txt",                   "Step 9 최종 보고서"),
]

for fpath, desc in output_files:
    exists = "✔️" if os.path.exists(f'./{fpath}') else "❌"
    try:
        size = os.path.getsize(f'./{fpath}')
        size_str = f"{size/1024:.1f} KB" if size > 1024 else f"{size} B"
    except:
        size_str = "N/A"
    print(f"  {exists} [{size_str:>8}]  {fpath:<45}  {desc}")

print("="*65)
print("\n🚀 전체 TinyML 파이프라인 완료! ESP32에 배포할 준비가 되었습니다.")
print("   auv_classifier_int8.h와 auv_norm_params.h를 Arduino 스케치 폴더에 복사하세요.")
