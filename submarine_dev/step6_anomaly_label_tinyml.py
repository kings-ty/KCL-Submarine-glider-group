import numpy as np
import os
import matplotlib
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 6: 이상 탐지 및 자동 라벨링
# Autoencoder 복원 오차(Reconstruction Error)를 이용해 이상 구간을 탐지하고,
# 물리적 규칙 기반(Rule-Based) 라벨러로 4개 클래스 자동 라벨링 수행
# ==============================================================================

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
import tensorflow as tf

# ─────────────────────────────────────────────
# [Cell 1] 사전 파일 로드
# ─────────────────────────────────────────────
print("\n[Step 1] 필요한 파일 로드 중...")

# 정규화된 전체 특징 벡터 로드 (N_windows × 30)
X_scaled = np.load("./ml-data/X_features_scaled.npy")

# 정규화 전 원시 특징 (라벨 규칙에 실제 값 범위 필요)
X_raw = np.load("./ml-data/X_features.npy")
df = pd.read_csv("./ml-data/step2_preprocessed_data.csv")

# 학습된 Autoencoder 로드
model_path = "./ml-data/autoencoder_model.keras"
if not os.path.exists(model_path):
    print("⚠️ 에러: autoencoder_model.keras 파일이 없습니다! Step 5를 먼저 실행하세요.")
    exit(1)

autoencoder = tf.keras.models.load_model(model_path)
print(
    f"✅ 로드 완료! 전체 윈도우: {X_scaled.shape[0]}개, 특징 차원: {X_scaled.shape[1]}"
)

# ─────────────────────────────────────────────
# [Cell 2] 전체 데이터에 대해 복원 오차(MSE) 계산
# ─────────────────────────────────────────────
print("\n[Step 2] 복원 오차(Reconstruction Error) 계산 중...")

# 모델 추론: 전체 데이터를 Autoencoder에 통과
X_reconstructed = autoencoder.predict(X_scaled, batch_size=256, verbose=0)

# 윈도우별 복원 MSE 계산 (각 Row의 30개 차원 평균)
recon_errors = np.mean((X_scaled - X_reconstructed) ** 2, axis=1)
print(f"✔️ 복원 오차 통계:")
print(f"   - Min:    {recon_errors.min():.6f}")
print(f"   - Mean:   {recon_errors.mean():.6f}")
print(f"   - Median: {np.median(recon_errors):.6f}")
print(f"   - Max:    {recon_errors.max():.6f}")

# ─────────────────────────────────────────────
# [Cell 3] 임계값(Threshold) 결정
# Training 복원 오차의 95th percentile을 anomaly threshold로 설정
# (사용자 지시: Step 5에서 정상 데이터로 학습했으므로
#  '정상 데이터의 복원 오차 분포' 기반 임계값 = 전체의 95th %ile)
# ─────────────────────────────────────────────
print("\n[Step 3 & 4] 3phase hybrid autonomous anomaly detection")

labels = np.zeros(len(X_scaled), dtype=np.int32)
WINDOW_SIZE = 64
STRIDE = 16

# ==============================
# Phase A: Physics
# ==============================
for i in range(len(X_scaled)):
    start_idx = i * STRIDE
    end_idx = start_idx + WINDOW_SIZE
    if end_idx > len(df):
        break
    window = df.iloc[start_idx:end_idx]
    fault_ratio = window["is_sensor_fault"].mean()

    if fault_ratio >= 0.50:
        labels[i] = 2

# ==============================
# Phase B: Autoencoder Reconstruction Error
# ==============================

channel_groups = {
    "ph": slice(0, 4),
    "EC": slice(4, 8),
    "DO": slice(8, 12),
    "Temp": slice(19, 22),
}
for i in range(len(X_scaled)):
    if labels[i] != 0:
        continue
    channel_errors = {}
    for ch_name, ch_slice in channel_groups.items():
        ch_error = np.mean((X_scaled[i, ch_slice] - X_reconstructed[i, ch_slice]) ** 2)
        channel_errors[ch_name] = ch_error

    max_ch = max(channel_errors, key=channel_errors.get)
    max_error = channel_errors[max_ch]
    other_errors = [v for k, v in channel_errors.items() if k != max_ch]
    avg_other = np.mean(other_errors) if other_errors else 0

    if max_error > avg_other * 5 and max_error > 0.05:
        labels[i] = 2
# ==============================
# Phase C: All reconstruction errors + env error rule
# ==============================
threshold_95 = np.percentile(recon_errors[labels == 0], 95)
print(f"✔️ 정상 윈도우 기반 Anomaly Threshold = {threshold_95:.6f}")

for i in range(len(X_scaled)):
    if labels[i] != 0:
        continue
    if recon_errors[i] <= threshold_95:
        labels[i] = 00
        continue
    # ── 임계값을 넘은 이상 징후 (ENV_ANOMALY vs SENSOR_FAULT 구분) ──
    # 원시 배열 X_raw 의 인덱스를 가져와 조건 비교 (step3_extract_features_tinyml.py 참고)
    ph_std = X_raw[i, 1]
    ec_std = X_raw[i, 5]
    dep_std = X_raw[i, 17]
    tmp_std = X_raw[i, 20]

    multi_sensor = (
        ph_std > 0.15  # 기존 0.2 → 0.15 낮춤
        and ec_std > 15  # 기존 20 → 15 낮춤
        and (dep_std > 0.2 or tmp_std > 0.3)
    )

    if multi_sensor:
        labels[i] = 3  # 다중 채널 변동은 환경적 이상(ENV_ANOMALY)
    else:
        labels[i] = 2  # 어느 규칙에도 속하지 않는 이상은 기본 SENSOR_FAULT로 처리
class_names = ["NORMAL", "MOTION_ARTIFACT", "SENSOR_FAULT", "ENV_ANOMALY"]
print("\n  📊 새로운 하이브리드 라벨 분포 결과:")
print(f"  {'Class':<20} {'Count':>8} {'Ratio':>8}")
print(f"  {'-' * 38}")
for i, name in enumerate(class_names):
    cnt = (labels == i).sum()
    ratio = cnt / len(labels) * 100
    print(f"  [{i}] {name:<17} {cnt:>8,} {ratio:>7.1f}%")
threshold = threshold_95  # (아래 차트 그릴 때 기존 변수명 호환을 위해)

# ─────────────────────────────────────────────
# [Cell 5] 라벨 분포 시각화 (Bar Chart)
# ─────────────────────────────────────────────
print("\n[Step 5] 라벨 분포 Bar Chart 저장...")

counts = [(labels == i).sum() for i in range(4)]
colors = ["#2ecc71", "#e74c3c", "#f39c12", "#3498db"]

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# --- 좌측: 절대 수량 bar ---
bars = axes[0].bar(class_names, counts, color=colors, edgecolor="black", linewidth=0.8)
axes[0].set_title("Label Distribution (Absolute Count)", fontsize=13, fontweight="bold")
axes[0].set_ylabel("Window Count")
axes[0].set_xlabel("Class")
for bar, cnt in zip(bars, counts):
    axes[0].text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + max(counts) * 0.01,
        f"{cnt:,}",
        ha="center",
        va="bottom",
        fontsize=10,
    )

# --- 우측: 복원 오차 히스토그램 + Threshold 표시 ---
axes[1].hist(
    recon_errors,
    bins=80,
    color="steelblue",
    alpha=0.7,
    edgecolor="black",
    linewidth=0.4,
)
axes[1].axvline(
    threshold,
    color="red",
    linewidth=2,
    linestyle="--",
    label=f"Threshold = {threshold:.4f}",
)
axes[1].set_title("Reconstruction Error Distribution", fontsize=13, fontweight="bold")
axes[1].set_xlabel("MSE Reconstruction Error")
axes[1].set_ylabel("Window Count")
axes[1].legend()

plt.tight_layout()
plt.savefig("./ml-data/label_distribution.png", dpi=150, bbox_inches="tight")
print("✔️ 라벨 분포 차트 저장 완료: ./ml-data/label_distribution.png")

# ─────────────────────────────────────────────
# [Cell 6] 결과 저장 (다음 Step 7에서 사용)
# ─────────────────────────────────────────────
print("\n[Step 6] 라벨 배열 저장 중...")

np.save("./ml-data/y_labels.npy", labels)
np.save("./ml-data/recon_errors.npy", recon_errors)
np.save("./ml-data/anomaly_threshold.npy", np.array([threshold]))

print("✔️ 저장 완료:")
print("   - ./ml-data/y_labels.npy        (각 윈도우 클래스 라벨 0~3)")
print("   - ./ml-data/recon_errors.npy    (각 윈도우 복원 MSE 오차)")
print("   - ./ml-data/anomaly_threshold.npy (임계값 스칼라)")

print("\n🎉 Step 6 완료! 자동 라벨링이 끝났습니다.")
print("   NORMAL/MOTION_ARTIFACT/SENSOR_FAULT/ENV_ANOMALY 4개 클래스로 분류됨.")
print("   다음 Step 7에서 이 라벨로 지도학습 Classifier를 훈련합니다.")
