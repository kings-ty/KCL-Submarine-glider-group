import numpy as np
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 5: Autoencoder 학습 (비지도 원포클래스 이상탐지)
# ==============================================================================

# TensorFlow GPU/CPU 경고 억제
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'
import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Dense
from tensorflow.keras.optimizers import Adam

# %% [Cell 1] 정규화된 데이터 로드
print("\n[Step 1] 정규화된 X_features_scaled.npy 로드 중...")
data_path = './ml-data/X_features_scaled.npy'
if not os.path.exists(data_path):
    print("⚠️ 에러: 정규화된 스케일 데이터 파일을 찾을 수 없습니다!")
    exit(1)

X_scaled = np.load(data_path)
print(f"✅ 데이터 로드 완료! 전체 윈도우 개수: {X_scaled.shape[0]}개 (차원: {X_scaled.shape[1]})")

# %% [Cell 2] "정상" 구간 필터링 (간단하게 Feature 들의 IQR 내에 있는 윈도우만 선택)
print("\n[Step 2] 비지도 학습용 '정상 데이터(Normal)' 필터링 (Outlier 분리)...")

# 30개의 Feature 각각에 대해 IQR 계산
Q1 = np.percentile(X_scaled, 25, axis=0)
Q3 = np.percentile(X_scaled, 75, axis=0)
IQR = Q3 - Q1

# 사용자 지시: IQR을 이용해 정상 윈도우만 추출 (1.5가 너무 엄격해 다 날아갈 수 있으니 3.0 IQR로 넉넉한 정상 범위 산정)
lower_bound = Q1 - 3.0 * IQR
upper_bound = Q3 + 3.0 * IQR

# 30개 파라미터가 전부 넉넉한 정상 바운더리 내에 있는 윈도우만 유지 (True/False 마스킹)
condition = (X_scaled >= lower_bound) & (X_scaled <= upper_bound)
normal_mask = np.all(condition, axis=1)

X_normal = X_scaled[normal_mask]
print(f"✔️ IQR 필터링 결과: {X_scaled.shape[0]}개 중 {X_normal.shape[0]}개 정상(Normal) 윈도우로 판별됨.")

if X_normal.shape[0] < 100:
    print("⚠️ 필터링이 너무 엄격해 정상 데이터가 100개 미만입니다! 예외적으로 전체 데이터를 '정상'으로 간주하고 학습합니다.")
    X_normal = X_scaled

# %% [Cell 3] Autoencoder 아키텍처 정의 (지시사항: 30 -> 16 -> 8 -> 16 -> 30)
print("\n[Step 3] Autoencoder 모델 아키텍처 빌드 (30 -> 16 -> 8 -> 16 -> 30)...")

input_dim = 30
inputs = Input(shape=(input_dim,), name="sensor_input")

# 인코더 (압축)
encoded = Dense(16, activation='relu', name="encoder_layer_1")(inputs)
encoded = Dense(8, activation='relu', name="bottleneck_8_latent")(encoded)

# 디코더 (복원) / 마지막은 정규화 범위(0~1)에 맞춰 sigmoid
decoded = Dense(16, activation='relu', name="decoder_layer_1")(encoded)
outputs = Dense(30, activation='sigmoid', name="reconstruction_output")(decoded)

autoencoder = Model(inputs, outputs, name="InWaterSense_Autoencoder")

# Loss: MSE, Optimizer: Adam
autoencoder.compile(optimizer=Adam(learning_rate=0.001), loss='mse')
autoencoder.summary()


# %% [Cell 4] Autoencoder 학습 시작 (Epochs 100, Batch 32, Val_split 15%)
print("\n[Step 4] 모델 학습(Training) 시작...")
history = autoencoder.fit(
    X_normal, X_normal, # 비지도 학습이므로 입력(X)을 줘서 출력(X)을 복원하도록 타겟도 X로 설정
    epochs=100,
    batch_size=32,
    validation_split=0.15,
    shuffle=True,
    verbose=1
)


# %% [Cell 5] 모델 저장 및 학습 곡선 시각화
print("\n[Step 5] 학습 곡선(Loss Curve) 시각화 및 모델 저장 중...")

# 케라스 모델 포맷으로 파일 저장
model_path = './ml-data/autoencoder_model.keras'
autoencoder.save(model_path)
print(f"✔️ 훈련된 모델 Keras 로컬 저장 완료: {model_path}")

# Loss 시각화 차트 그려서 저장
plt.figure(figsize=(10, 6))
plt.plot(history.history['loss'], label='Training Loss (MSE)', color='blue')
plt.plot(history.history['val_loss'], label='Validation Loss (MSE)', color='orange')
plt.title('Autoencoder Reconstruction Loss Progress')
plt.xlabel('Epochs')
plt.ylabel('Mean Squared Error (MSE)')
plt.legend()
plt.grid(True, linestyle='--', alpha=0.6)

plot_path = './ml-data/autoencoder_loss_curve.png'
plt.savefig(plot_path)
print(f"✔️ 학습 곡선 스냅샷 저장 완료: {plot_path}")

print("\n🎉 Step 5: (비지도) 오토인코더 훈련이 완벽히 끝났습니다!")
print("이상탐지 추론(Inference) 단계에서는 새 윈도우를 이 모델에 통과시키고,")
print("입력과 출력의 차이(Reconstruction MSE Error)가 특정 임계치(Threshold)를 넘으면 이상 상황으로 분리하면 됩니다.")
print("이제 TFLite 원시 변환 기법(Tensorflow Lite Micro)을 통해 ESP32용 C배열(.h) 모델로 추출할 준비가 되었습니다.")
