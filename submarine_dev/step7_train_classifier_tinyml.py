import numpy as np
import os
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import seaborn as sns

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 7: Classifier 학습 (지도학습 4-class)
# Step 6의 자동 라벨을 이용해 30개 특징 → 4개 클래스 분류 모델 훈련
# 아키텍처: 30 → 32(relu) → 16(relu) → 4(softmax)
# ==============================================================================

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix

# ─────────────────────────────────────────────
# [Cell 1] 데이터 로드
# ─────────────────────────────────────────────
print("\n[Step 1] Step 6 결과 파일 로드 중...")

X = np.load("./ml-data/X_features_scaled.npy")  # 정규화된 특징
y = np.load("./ml-data/y_labels.npy")  # 자동 라벨 (0~3)

print(f"✅ X shape: {X.shape}, y shape: {y.shape}")

# 클래스별 샘플 수 확인
class_names = ["NORMAL", "MOTION_ARTIFACT", "SENSOR_FAULT", "ENV_ANOMALY"]
for i, name in enumerate(class_names):
    cnt = (y == i).sum()
    print(f"   [{i}] {name}: {cnt:,}개 ({100 * cnt / len(y):.1f}%)")

# 특정 클래스가 하나도 없을 경우 경고 처리
unique_classes = np.unique(y)
if len(unique_classes) < 4:
    print(f"\n⚠️  주의: 현재 데이터에 {4 - len(unique_classes)}개 클래스가 없습니다.")
    print(f"    실제 존재 클래스: {[class_names[i] for i in unique_classes]}")
    print("    (InWaterSense는 IMU/depth 없어 일부 클래스 비어있을 수 있음 – 정상)")

# ─────────────────────────────────────────────
# [Cell 2] Train/Test Split (80/20, Stratified)
# Stratified: 각 클래스 비율을 train/test에 동일하게 유지
# ─────────────────────────────────────────────
print("\n[Step 2] Train/Test 분할 (80%/20%, Stratified)...")

# stratify 파라미터로 클래스 불균형 보완
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42, stratify=y
)

print(f"✔️ Train: {X_train.shape[0]:,}개 | Test: {X_test.shape[0]:,}개")

# from sklearn.utils.class_weight import compute_class_weight

# weights = compute_class_weight("balanced", classes=np.unique(y_train), y=y_train)
# class_weight_dict = dict(zip(np.unique(y_train), weights))
# print(f"Class weights: {class_weight_dict}")
# SENSOR_FAULT(2)에는 2배, ENV_ANOMALY(3)에는 3배의 오답 패널티를 주는 수동 가중치 설정
class_weight_dict = {
    0: 1.0,  # NORMAL (기본)
    1: 1.0,  # MOTION_ARTIFACT (비어있더라도 적어주는 것이 안전함)
    2: 2.0,  # SENSOR_FAULT (오답 시 2배 징벌)
    3: 3.0,  # ENV_ANOMALY  (오답 시 3배 징벌)
}

print(f"수동 설정된 Class weights: {class_weight_dict}")

# One-hot encoding (TF/Keras Categorical Crossentropy용)
y_train_oh = tf.keras.utils.to_categorical(y_train, num_classes=4)
y_test_oh = tf.keras.utils.to_categorical(y_test, num_classes=4)

# ─────────────────────────────────────────────
# [Cell 3] Classifier 모델 정의
# 지시사항: 30 → 32(relu, dropout 0.2) → 16(relu, dropout 0.1) → 4(softmax)
# ─────────────────────────────────────────────
print("\n[Step 3] Classifier 아키텍처 빌드...")

classifier = Sequential(
    [
        Dense(32, activation="relu", input_shape=(30,), name="hidden_layer_1"),
        Dropout(0.2, name="dropout_1"),  # 과적합 방지 (20%)
        Dense(16, activation="relu", name="hidden_layer_2"),
        Dropout(0.1, name="dropout_2"),  # 과적합 방지 (10%)
        Dense(4, activation="softmax", name="output_layer"),  # 4개 클래스 확률
    ],
    name="AUV_4class_Classifier",
)

classifier.compile(
    optimizer=Adam(learning_rate=0.001),
    loss="categorical_crossentropy",
    metrics=["accuracy"],
)

classifier.summary()

# ─────────────────────────────────────────────
# [Cell 4] 콜백 정의
# EarlyStopping: val_loss 5 epoch 동안 개선 없으면 중단
# ReduceLROnPlateau: val_loss 3 epoch 동안 정체시 LR 0.5배 감소
# ─────────────────────────────────────────────
callbacks = [
    EarlyStopping(
        monitor="val_loss", patience=15, restore_best_weights=True, verbose=1
    ),
    ReduceLROnPlateau(
        monitor="val_loss", factor=0.5, patience=5, min_lr=1e-6, verbose=1
    ),
]

# ─────────────────────────────────────────────
# [Cell 5] 모델 학습 (Epochs 80, Batch 32, Val 15%)
# ─────────────────────────────────────────────
print("\n[Step 4] 분류 모델 학습 시작 (최대 80 Epochs, Batch=32)...")

history = classifier.fit(
    X_train,
    y_train_oh,
    epochs=80,
    batch_size=32,
    validation_split=0.15,  # 훈련 데이터의 15% 검증용
    callbacks=callbacks,
    class_weight=class_weight_dict,
    shuffle=True,
    verbose=1,
)

# ─────────────────────────────────────────────
# [Cell 6] 학습 곡선 시각화 저장
# ─────────────────────────────────────────────
print("\n[Step 5] 학습 곡선 시각화 저장...")

fig, axes = plt.subplots(1, 2, figsize=(14, 5))

# Loss 곡선
axes[0].plot(history.history["loss"], label="Train Loss", color="blue")
axes[0].plot(history.history["val_loss"], label="Val Loss", color="orange")
axes[0].set_title("Classifier Training Loss", fontweight="bold")
axes[0].set_xlabel("Epoch")
axes[0].set_ylabel("Categorical Crossentropy")
axes[0].legend()
axes[0].grid(True, linestyle="--", alpha=0.5)

# Accuracy 곡선
axes[1].plot(history.history["accuracy"], label="Train Accuracy", color="green")
axes[1].plot(history.history["val_accuracy"], label="Val Accuracy", color="red")
axes[1].set_title("Classifier Training Accuracy", fontweight="bold")
axes[1].set_xlabel("Epoch")
axes[1].set_ylabel("Accuracy")
axes[1].legend()
axes[1].grid(True, linestyle="--", alpha=0.5)

plt.tight_layout()
plt.savefig("./ml-data/classifier_training_curve.png", dpi=150, bbox_inches="tight")
print("✔️ 학습 곡선 저장: ./ml-data/classifier_training_curve.png")

# ─────────────────────────────────────────────
# [Cell 7] 테스트 데이터 평가 및 보고서 출력
# ─────────────────────────────────────────────
print("\n[Step 6] Test 데이터 성능 평가...")

# 예측 (softmax → argmax로 클래스 결정)
y_pred_prob = classifier.predict(X_test, verbose=0)
y_pred = np.argmax(y_pred_prob, axis=1)

# 실제 존재하는 클래스 이름만 추출 (비어있는 클래스 제외하여 report 오류 방지)
present_classes = sorted(np.unique(np.concatenate([y_test, y_pred])))
target_names = [class_names[i] for i in present_classes]

# Classification Report (precision, recall, F1 per class)
report = classification_report(
    y_test, y_pred, labels=present_classes, target_names=target_names, zero_division=0
)
print("\n📋 Classification Report:")
print(report)

# 전체 Accuracy
from sklearn.metrics import accuracy_score

overall_acc = accuracy_score(y_test, y_pred)
print(f"✔️ Overall Accuracy: {overall_acc * 100:.2f}%")

# ─────────────────────────────────────────────
# [Cell 8] Confusion Matrix 시각화 저장
# ─────────────────────────────────────────────
print("\n[Step 7] Confusion Matrix 시각화 저장...")

cm = confusion_matrix(y_test, y_pred, labels=present_classes)

fig, ax = plt.subplots(figsize=(7, 6))
sns.heatmap(
    cm,
    annot=True,
    fmt="d",
    cmap="Blues",
    xticklabels=target_names,
    yticklabels=target_names,
    linewidths=0.5,
    linecolor="gray",
    ax=ax,
)
ax.set_xlabel("Predicted Label", fontsize=12)
ax.set_ylabel("True Label", fontsize=12)
ax.set_title(
    "Confusion Matrix - AUV 4-Class Classifier", fontsize=13, fontweight="bold"
)
plt.xticks(rotation=30, ha="right")
plt.tight_layout()

plt.savefig("./confusion_matrix.png", dpi=150, bbox_inches="tight")
print("✔️ Confusion Matrix 저장: ./confusion_matrix.png")

# ─────────────────────────────────────────────
# [Cell 9] 모델 저장
# ─────────────────────────────────────────────
print("\n[Step 8] 분류 모델 저장 중...")

classifier.save("./ml-data/classifier_model.keras")
print("✔️ 분류 모델 저장: ./ml-data/classifier_model.keras")

# Step 8에서 사용할 성능 지표를 파일로 저장
from sklearn.metrics import f1_score

f1_per_class = f1_score(
    y_test, y_pred, labels=present_classes, average=None, zero_division=0
)
f1_macro = f1_score(y_test, y_pred, average="macro", zero_division=0)

np.save("./ml-data/fp32_metrics.npy", np.array([overall_acc, f1_macro]))
np.save("./ml-data/fp32_f1_per_class.npy", f1_per_class)

print(f"\n🎉 Step 7 완료! Classifier 훈련 및 평가가 끝났습니다.")
print(f"   Test Accuracy: {overall_acc * 100:.2f}%  |  F1 (macro): {f1_macro:.4f}")
print("   다음 Step 8에서 TFLite INT8 양자화를 수행합니다.")
