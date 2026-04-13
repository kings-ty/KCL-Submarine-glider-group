"""
============================================================================
 AUV TinyML Training Pipeline
 train_auv_model.py
============================================================================

 실행: python train_auv_model.py
 출력: 
   - auv_autoencoder_model.h   (INT8 quantized TFLite → C array)
   - auv_model_f32.h           (FP32 TFLite → C array, 비교용)
   - training_report.txt        (모든 수치: accuracy, latency 예측, 모델 크기)
   - confusion_matrix.png       (시각화)

 요구 라이브러리:
   pip install tensorflow numpy scikit-learn matplotlib

============================================================================
"""

import numpy as np
import tensorflow as tf
from tensorflow import keras
from sklearn.model_selection import train_test_split
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import time
import os

# ============================================================
# 1. 데이터 생성 (나중에 실제 SD카드 CSV로 교체)
# ============================================================
# 
# 📌 실제 데이터가 생기면 이 섹션만 교체:
#    df = pd.read_csv("auv_log.csv")
#    features = extract_features_from_windows(df)  # 30개 특징
#    labels = df['ground_truth'].values

def generate_synthetic_data(n_normal=2000, n_fault=400, n_anomaly=300, n_motion=300):
    """
    합성 데이터 생성 — 실제 데이터 수집 전 파이프라인 테스트용
    
    30개 특징:
    [0-3]   pH:    mean, std, slope, rms
    [4-7]   EC:    mean, std, slope, rms
    [8-11]  DO:    mean, std, slope, rms
    [12-15] O2:    mean, std, slope, rms
    [16-18] Depth: mean, std, slope
    [19-21] Temp:  mean, std, slope
    [22-24] IMU:   energy_mean, energy_std, energy_max
    [25]    pH-Temp correlation
    [26-27] pH residual: mean, std
    [28-29] DO residual: mean, std
    """
    np.random.seed(42)
    
    # ── CLASS 0: NORMAL ──
    # 안정적인 센서 값, 낮은 std, 낮은 slope, 잔차 작음
    normal = np.column_stack([
        np.random.normal(0.45, 0.02, n_normal),   # pH mean (voltage)
        np.random.normal(0.005, 0.002, n_normal),  # pH std (작음)
        np.random.normal(0.0, 0.001, n_normal),    # pH slope (거의 0)
        np.random.normal(0.45, 0.02, n_normal),    # pH rms
        np.random.normal(0.80, 0.03, n_normal),    # EC mean
        np.random.normal(0.008, 0.003, n_normal),  # EC std
        np.random.normal(0.0, 0.001, n_normal),    # EC slope
        np.random.normal(0.80, 0.03, n_normal),    # EC rms
        np.random.normal(1.20, 0.04, n_normal),    # DO mean
        np.random.normal(0.01, 0.004, n_normal),   # DO std
        np.random.normal(0.0, 0.001, n_normal),    # DO slope
        np.random.normal(1.20, 0.04, n_normal),    # DO rms
        np.random.normal(0.0, 0.001, n_normal),    # O2 mean (비활성)
        np.random.normal(0.0, 0.001, n_normal),    # O2 std
        np.random.normal(0.0, 0.0005, n_normal),   # O2 slope
        np.random.normal(0.0, 0.001, n_normal),    # O2 rms
        np.random.normal(3.0, 0.5, n_normal),      # Depth mean
        np.random.normal(0.1, 0.05, n_normal),     # Depth std
        np.random.normal(0.0, 0.01, n_normal),     # Depth slope
        np.random.normal(20.0, 1.0, n_normal),     # Temp mean
        np.random.normal(0.1, 0.05, n_normal),     # Temp std
        np.random.normal(0.0, 0.005, n_normal),    # Temp slope
        np.random.normal(0.2, 0.1, n_normal),      # IMU energy mean (안정)
        np.random.normal(0.05, 0.02, n_normal),    # IMU energy std
        np.random.normal(0.5, 0.2, n_normal),      # IMU energy max
        np.random.normal(-0.02, 0.05, n_normal),   # pH-Temp corr
        np.random.normal(0.0, 0.05, n_normal),     # pH residual mean (작음!)
        np.random.normal(0.02, 0.01, n_normal),    # pH residual std
        np.random.normal(0.0, 0.05, n_normal),     # DO residual mean (작음!)
        np.random.normal(0.03, 0.01, n_normal),    # DO residual std
    ])
    
    # ── CLASS 1: MOTION ARTIFACT ──
    # IMU energy 높음, WQ std 높지만 잔차는 정상
    motion = np.column_stack([
        np.random.normal(0.45, 0.04, n_motion),    # pH mean (약간 불안정)
        np.random.normal(0.03, 0.01, n_motion),    # pH std (높음!)
        np.random.normal(0.0, 0.005, n_motion),    # pH slope
        np.random.normal(0.46, 0.04, n_motion),    # pH rms
        np.random.normal(0.80, 0.05, n_motion),    # EC mean
        np.random.normal(0.04, 0.015, n_motion),   # EC std (높음!)
        np.random.normal(0.0, 0.003, n_motion),    # EC slope
        np.random.normal(0.81, 0.05, n_motion),    # EC rms
        np.random.normal(1.20, 0.06, n_motion),    # DO mean
        np.random.normal(0.04, 0.015, n_motion),   # DO std (높음!)
        np.random.normal(0.0, 0.003, n_motion),    # DO slope
        np.random.normal(1.21, 0.06, n_motion),    # DO rms
        np.random.normal(0.0, 0.001, n_motion),    # O2 mean
        np.random.normal(0.0, 0.001, n_motion),    # O2 std
        np.random.normal(0.0, 0.0005, n_motion),   # O2 slope
        np.random.normal(0.0, 0.001, n_motion),    # O2 rms
        np.random.normal(3.5, 1.0, n_motion),      # Depth mean
        np.random.normal(0.8, 0.3, n_motion),      # Depth std (움직임!)
        np.random.normal(0.0, 0.05, n_motion),     # Depth slope
        np.random.normal(20.0, 1.0, n_motion),     # Temp mean
        np.random.normal(0.15, 0.05, n_motion),    # Temp std
        np.random.normal(0.0, 0.005, n_motion),    # Temp slope
        np.random.normal(3.0, 1.0, n_motion),      # IMU energy mean (높음!!)
        np.random.normal(1.0, 0.4, n_motion),      # IMU energy std (높음!)
        np.random.normal(5.0, 2.0, n_motion),      # IMU energy max (높음!)
        np.random.normal(-0.02, 0.08, n_motion),   # pH-Temp corr
        np.random.normal(0.0, 0.08, n_motion),     # pH residual mean (정상)
        np.random.normal(0.03, 0.015, n_motion),   # pH residual std
        np.random.normal(0.0, 0.08, n_motion),     # DO residual mean (정상)
        np.random.normal(0.04, 0.015, n_motion),   # DO residual std
    ])
    
    # ── CLASS 2: SENSOR FAULT (biofouling) ──
    # pH가 drift하면서 잔차 크게 벌어짐, 온도/수심은 안정적
    fault = np.column_stack([
        np.random.normal(0.55, 0.06, n_fault),     # pH mean (drift!)
        np.random.normal(0.02, 0.01, n_fault),     # pH std
        np.random.normal(0.02, 0.008, n_fault),    # pH slope (drift!)
        np.random.normal(0.56, 0.06, n_fault),     # pH rms (높음)
        np.random.normal(0.80, 0.03, n_fault),     # EC mean (정상)
        np.random.normal(0.008, 0.003, n_fault),   # EC std
        np.random.normal(0.0, 0.001, n_fault),     # EC slope
        np.random.normal(0.80, 0.03, n_fault),     # EC rms
        np.random.normal(1.20, 0.04, n_fault),     # DO mean
        np.random.normal(0.01, 0.004, n_fault),    # DO std
        np.random.normal(0.0, 0.001, n_fault),     # DO slope
        np.random.normal(1.20, 0.04, n_fault),     # DO rms
        np.random.normal(0.0, 0.001, n_fault),     # O2 mean
        np.random.normal(0.0, 0.001, n_fault),     # O2 std
        np.random.normal(0.0, 0.0005, n_fault),    # O2 slope
        np.random.normal(0.0, 0.001, n_fault),     # O2 rms
        np.random.normal(3.0, 0.5, n_fault),       # Depth mean (안정)
        np.random.normal(0.1, 0.05, n_fault),      # Depth std (안정)
        np.random.normal(0.0, 0.01, n_fault),      # Depth slope
        np.random.normal(20.0, 0.5, n_fault),      # Temp mean (안정!)
        np.random.normal(0.08, 0.03, n_fault),     # Temp std (안정!)
        np.random.normal(0.0, 0.003, n_fault),     # Temp slope
        np.random.normal(0.2, 0.1, n_fault),       # IMU energy mean (안정)
        np.random.normal(0.05, 0.02, n_fault),     # IMU energy std
        np.random.normal(0.5, 0.2, n_fault),       # IMU energy max
        np.random.normal(0.1, 0.15, n_fault),      # pH-Temp corr (깨짐!)
        np.random.normal(0.8, 0.3, n_fault),       # pH residual mean (크다!!)
        np.random.normal(0.15, 0.06, n_fault),     # pH residual std (크다!)
        np.random.normal(0.0, 0.05, n_fault),      # DO residual mean (정상)
        np.random.normal(0.03, 0.01, n_fault),     # DO residual std
    ])
    
    # ── CLASS 3: ENVIRONMENTAL ANOMALY ──
    # 모든 WQ 센서가 동시에 변동 + depth/temp 변화 동반
    anomaly = np.column_stack([
        np.random.normal(0.38, 0.05, n_anomaly),   # pH mean (떨어짐)
        np.random.normal(0.04, 0.015, n_anomaly),  # pH std (높음)
        np.random.normal(-0.01, 0.005, n_anomaly), # pH slope (하강!)
        np.random.normal(0.39, 0.05, n_anomaly),   # pH rms
        np.random.normal(0.95, 0.08, n_anomaly),   # EC mean (올라감!)
        np.random.normal(0.03, 0.01, n_anomaly),   # EC std (높음)
        np.random.normal(0.005, 0.003, n_anomaly), # EC slope (상승!)
        np.random.normal(0.96, 0.08, n_anomaly),   # EC rms
        np.random.normal(0.90, 0.08, n_anomaly),   # DO mean (떨어짐!)
        np.random.normal(0.05, 0.02, n_anomaly),   # DO std (높음)
        np.random.normal(-0.005, 0.003, n_anomaly),# DO slope (하강!)
        np.random.normal(0.91, 0.08, n_anomaly),   # DO rms
        np.random.normal(0.0, 0.001, n_anomaly),   # O2 mean
        np.random.normal(0.0, 0.001, n_anomaly),   # O2 std
        np.random.normal(0.0, 0.0005, n_anomaly),  # O2 slope
        np.random.normal(0.0, 0.001, n_anomaly),   # O2 rms
        np.random.normal(5.0, 2.0, n_anomaly),     # Depth mean (다름!)
        np.random.normal(0.8, 0.3, n_anomaly),     # Depth std (변동!)
        np.random.normal(0.05, 0.02, n_anomaly),   # Depth slope
        np.random.normal(18.0, 2.0, n_anomaly),    # Temp mean (다름!)
        np.random.normal(0.5, 0.2, n_anomaly),     # Temp std (변동!)
        np.random.normal(-0.01, 0.005, n_anomaly), # Temp slope
        np.random.normal(0.3, 0.15, n_anomaly),    # IMU energy mean
        np.random.normal(0.08, 0.03, n_anomaly),   # IMU energy std
        np.random.normal(0.7, 0.3, n_anomaly),     # IMU energy max
        np.random.normal(-0.3, 0.15, n_anomaly),   # pH-Temp corr (상관관계 있음!)
        np.random.normal(0.3, 0.15, n_anomaly),    # pH residual mean (중간)
        np.random.normal(0.08, 0.03, n_anomaly),   # pH residual std
        np.random.normal(0.4, 0.15, n_anomaly),    # DO residual mean (중간)
        np.random.normal(0.08, 0.03, n_anomaly),   # DO residual std
    ])
    
    X = np.vstack([normal, motion, fault, anomaly]).astype(np.float32)
    y = np.concatenate([
        np.zeros(n_normal),
        np.ones(n_motion),
        np.full(n_fault, 2),
        np.full(n_anomaly, 3)
    ]).astype(np.int32)
    
    return X, y


# ============================================================
# 2. 데이터 정규화
# ============================================================
def normalize_data(X_train, X_test):
    """Min-Max 정규화 — ESP32에서도 동일하게 적용해야 함"""
    mins = X_train.min(axis=0)
    maxs = X_train.max(axis=0)
    ranges = maxs - mins
    ranges[ranges == 0] = 1.0  # 0으로 나누기 방지
    
    X_train_norm = (X_train - mins) / ranges
    X_test_norm = (X_test - mins) / ranges
    
    return X_train_norm, X_test_norm, mins, ranges


# ============================================================
# 3. 모델 A: Autoencoder (비지도 — 이상 탐지)
# ============================================================
def build_autoencoder(input_dim=30):
    """
    30 → 16 → 8 → 16 → 30 Autoencoder
    
    정상 데이터만으로 학습.
    복원 오차(MSE) > threshold → 이상
    """
    encoder_input = keras.Input(shape=(input_dim,))
    x = keras.layers.Dense(16, activation='relu')(encoder_input)
    x = keras.layers.Dense(8, activation='relu')(x)       # Bottleneck
    x = keras.layers.Dense(16, activation='relu')(x)
    decoder_output = keras.layers.Dense(input_dim, activation='sigmoid')(x)
    
    autoencoder = keras.Model(encoder_input, decoder_output, name='auv_autoencoder')
    autoencoder.compile(optimizer='adam', loss='mse')
    
    return autoencoder


# ============================================================
# 4. 모델 B: Classifier (지도 — 4-class)
# ============================================================
def build_classifier(input_dim=30, num_classes=4):
    """
    30 → 32 → 16 → 4 Classifier
    
    라벨링된 데이터로 학습.
    직접 4개 클래스 분류.
    """
    model = keras.Sequential([
        keras.layers.Input(shape=(input_dim,)),
        keras.layers.Dense(32, activation='relu'),
        keras.layers.Dropout(0.2),
        keras.layers.Dense(16, activation='relu'),
        keras.layers.Dropout(0.1),
        keras.layers.Dense(num_classes, activation='softmax')
    ], name='auv_classifier')
    
    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )
    return model


# ============================================================
# 5. INT8 Quantization
# ============================================================
def quantize_model(model, X_calibration, output_name):
    """
    FP32 → INT8 양자화
    
    Args:
        model: 학습된 Keras 모델
        X_calibration: 대표 데이터 (양자화 범위 결정용)
        output_name: 출력 파일 이름 접두사
    
    Returns:
        (fp32_model_bytes, int8_model_bytes, fp32_size, int8_size)
    """
    
    # ── FP32 TFLite 변환 ──
    converter_f32 = tf.lite.TFLiteConverter.from_keras_model(model)
    tflite_f32 = converter_f32.convert()
    fp32_size = len(tflite_f32)
    
    # ── INT8 Full Quantization ──
    def representative_dataset():
        for i in range(min(200, len(X_calibration))):
            yield [X_calibration[i:i+1].astype(np.float32)]
    
    converter_int8 = tf.lite.TFLiteConverter.from_keras_model(model)
    converter_int8.optimizations = [tf.lite.Optimize.DEFAULT]
    converter_int8.representative_dataset = representative_dataset
    converter_int8.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter_int8.inference_input_type = tf.int8
    converter_int8.inference_output_type = tf.int8
    tflite_int8 = converter_int8.convert()
    int8_size = len(tflite_int8)
    
    print(f"\n  📦 Model Size Comparison:")
    print(f"     FP32:  {fp32_size:,} bytes ({fp32_size/1024:.1f} KB)")
    print(f"     INT8:  {int8_size:,} bytes ({int8_size/1024:.1f} KB)")
    print(f"     Compression: {fp32_size/int8_size:.1f}x smaller")
    
    return tflite_f32, tflite_int8, fp32_size, int8_size


# ============================================================
# 6. C Header Export
# ============================================================
def export_to_c_header(tflite_bytes, filename, var_name):
    """TFLite 모델을 C 배열로 변환 → .h 파일"""
    
    with open(filename, 'w') as f:
        f.write(f"// Auto-generated by train_auv_model.py\n")
        f.write(f"// Model size: {len(tflite_bytes)} bytes\n")
        f.write(f"// Quantization: {'INT8' if 'int8' in filename else 'FP32'}\n\n")
        f.write(f"#ifndef {var_name.upper()}_H\n")
        f.write(f"#define {var_name.upper()}_H\n\n")
        f.write(f"const unsigned int {var_name}_len = {len(tflite_bytes)};\n")
        f.write(f"alignas(16) const unsigned char {var_name}[] = {{\n")
        
        for i, byte in enumerate(tflite_bytes):
            if i % 16 == 0:
                f.write("  ")
            f.write(f"0x{byte:02x}")
            if i < len(tflite_bytes) - 1:
                f.write(", ")
            if (i + 1) % 16 == 0:
                f.write("\n")
        
        f.write("\n};\n\n")
        f.write(f"#endif // {var_name.upper()}_H\n")
    
    print(f"  ✅ Exported: {filename} ({len(tflite_bytes)} bytes)")


# ============================================================
# 7. 정규화 파라미터 Export (ESP32에서 동일 정규화 필요)
# ============================================================
def export_normalization_params(mins, ranges, filename="auv_norm_params.h"):
    """Min-Max 정규화 파라미터를 C 헤더로 내보내기"""
    
    with open(filename, 'w') as f:
        f.write("// Auto-generated normalization parameters\n")
        f.write("// Apply BEFORE inference: normalized = (raw - min) / range\n\n")
        f.write("#ifndef AUV_NORM_PARAMS_H\n")
        f.write("#define AUV_NORM_PARAMS_H\n\n")
        f.write(f"#define NUM_FEATURES {len(mins)}\n\n")
        
        f.write("const float feature_mins[NUM_FEATURES] = {\n  ")
        f.write(", ".join([f"{v:.6f}f" for v in mins]))
        f.write("\n};\n\n")
        
        f.write("const float feature_ranges[NUM_FEATURES] = {\n  ")
        f.write(", ".join([f"{v:.6f}f" for v in ranges]))
        f.write("\n};\n\n")
        
        f.write("#endif // AUV_NORM_PARAMS_H\n")
    
    print(f"  ✅ Exported: {filename}")


# ============================================================
# 8. TFLite 정확도 검증 (양자화 후 성능 저하 측정)
# ============================================================
def evaluate_tflite_model(tflite_bytes, X_test, y_test, is_classifier=True):
    """TFLite 모델로 직접 inference해서 정확도 측정"""
    
    interpreter = tf.lite.Interpreter(model_content=tflite_bytes)
    interpreter.allocate_tensors()
    
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    
    input_scale = input_details[0].get('quantization_parameters', {}).get('scales', [1.0])
    input_zp = input_details[0].get('quantization_parameters', {}).get('zero_points', [0])
    
    predictions = []
    latencies = []
    
    for i in range(len(X_test)):
        sample = X_test[i:i+1].astype(np.float32)
        
        # INT8이면 양자화
        if input_details[0]['dtype'] == np.int8:
            sample = (sample / input_scale[0] + input_zp[0]).astype(np.int8)
        
        interpreter.set_tensor(input_details[0]['index'], sample)
        
        start = time.perf_counter()
        interpreter.invoke()
        latencies.append((time.perf_counter() - start) * 1000)  # ms
        
        output = interpreter.get_tensor(output_details[0]['index'])
        
        if is_classifier:
            predictions.append(np.argmax(output))
        else:
            # Autoencoder: reconstruction error
            if output_details[0]['dtype'] == np.int8:
                out_scale = output_details[0]['quantization_parameters']['scales'][0]
                out_zp = output_details[0]['quantization_parameters']['zero_points'][0]
                output = (output.astype(np.float32) - out_zp) * out_scale
            mse = np.mean((sample.astype(np.float32) - output) ** 2)
            predictions.append(mse)
    
    avg_latency = np.mean(latencies)
    
    if is_classifier:
        accuracy = np.mean(np.array(predictions) == y_test) * 100
        return accuracy, avg_latency, predictions
    else:
        return predictions, avg_latency


# ============================================================
# 📊 MAIN: 전체 파이프라인 실행
# ============================================================
def main():
    print("=" * 60)
    print("  AUV TinyML Training Pipeline")
    print("=" * 60)
    
    # ── 1. 데이터 준비 ──
    print("\n[1/7] Generating synthetic data...")
    X, y = generate_synthetic_data()
    print(f"  Total samples: {len(X)}")
    print(f"  Class distribution: {np.bincount(y)}")
    print(f"  Feature dimension: {X.shape[1]}")
    
    X_train, X_test, y_train, y_test = train_test_split(
        X, y, test_size=0.2, random_state=42, stratify=y
    )
    
    # ── 2. 정규화 ──
    print("\n[2/7] Normalizing data...")
    X_train_norm, X_test_norm, mins, ranges = normalize_data(X_train, X_test)
    export_normalization_params(mins, ranges)
    
    # ── 3. Autoencoder 학습 (정상 데이터만) ──
    print("\n[3/7] Training Autoencoder (normal data only)...")
    X_train_normal = X_train_norm[y_train == 0]
    
    autoencoder = build_autoencoder(input_dim=30)
    autoencoder.summary()
    
    history_ae = autoencoder.fit(
        X_train_normal, X_train_normal,
        epochs=100,
        batch_size=32,
        validation_split=0.15,
        verbose=1
    )
    
    # Autoencoder threshold 결정
    train_recon = autoencoder.predict(X_train_normal, verbose=0)
    train_mse = np.mean((X_train_normal - train_recon) ** 2, axis=1)
    ae_threshold = np.percentile(train_mse, 95)
    print(f"  Anomaly threshold (95th percentile): {ae_threshold:.6f}")
    
    # ── 4. Classifier 학습 (전체 데이터) ──
    print("\n[4/7] Training 4-class Classifier...")
    classifier = build_classifier(input_dim=30, num_classes=4)
    classifier.summary()
    
    history_cls = classifier.fit(
        X_train_norm, y_train,
        epochs=80,
        batch_size=32,
        validation_split=0.15,
        verbose=1
    )
    
    # FP32 정확도
    _, fp32_accuracy = classifier.evaluate(X_test_norm, y_test, verbose=0)
    y_pred_fp32 = np.argmax(classifier.predict(X_test_norm, verbose=0), axis=1)
    
    print(f"\n  FP32 Classifier Accuracy: {fp32_accuracy*100:.2f}%")
    print("\n  Classification Report (FP32):")
    class_names = ['NORMAL', 'MOTION_ART', 'SENSOR_FAULT', 'ENV_ANOMALY']
    print(classification_report(y_test, y_pred_fp32, target_names=class_names))
    
    # ── 5. INT8 Quantization ──
    print("\n[5/7] INT8 Quantization...")
    
    print("\n  --- Autoencoder ---")
    ae_f32, ae_int8, ae_f32_size, ae_int8_size = quantize_model(
        autoencoder, X_train_normal, "autoencoder"
    )
    
    print("\n  --- Classifier ---")
    cls_f32, cls_int8, cls_f32_size, cls_int8_size = quantize_model(
        classifier, X_train_norm, "classifier"
    )
    
    # ── 6. C 헤더 파일 내보내기 ──
    print("\n[6/7] Exporting C headers...")
    export_to_c_header(ae_int8,  "auv_autoencoder_int8.h",  "auv_ae_model")
    export_to_c_header(ae_f32,   "auv_autoencoder_f32.h",   "auv_ae_model_f32")
    export_to_c_header(cls_int8, "auv_classifier_int8.h",   "auv_cls_model")
    export_to_c_header(cls_f32,  "auv_classifier_f32.h",    "auv_cls_model_f32")
    
    # ── 7. 양자화 후 정확도 검증 ──
    print("\n[7/7] Post-quantization accuracy validation...")
    
    # Classifier: FP32 vs INT8
    cls_f32_acc, cls_f32_lat, _ = evaluate_tflite_model(cls_f32, X_test_norm, y_test)
    cls_int8_acc, cls_int8_lat, y_pred_int8 = evaluate_tflite_model(cls_int8, X_test_norm, y_test)
    
    # ============================================================
    # 📊 최종 보고서 출력
    # ============================================================
    report = []
    report.append("=" * 60)
    report.append("  AUV TinyML — THESIS DEFENSE METRICS")
    report.append("=" * 60)
    report.append("")
    report.append("┌─────────────────────────────────────────────────────┐")
    report.append("│  MODEL SIZE COMPARISON                              │")
    report.append("├──────────────────┬──────────┬──────────┬────────────┤")
    report.append("│ Model            │ FP32     │ INT8     │ Reduction  │")
    report.append("├──────────────────┼──────────┼──────────┼────────────┤")
    report.append(f"│ Autoencoder      │ {ae_f32_size/1024:6.1f} KB │ {ae_int8_size/1024:6.1f} KB │ {ae_f32_size/ae_int8_size:5.1f}x      │")
    report.append(f"│ Classifier       │ {cls_f32_size/1024:6.1f} KB │ {cls_int8_size/1024:6.1f} KB │ {cls_f32_size/cls_int8_size:5.1f}x      │")
    report.append("└──────────────────┴──────────┴──────────┴────────────┘")
    report.append("")
    report.append("┌─────────────────────────────────────────────────────┐")
    report.append("│  CLASSIFIER ACCURACY                                │")
    report.append("├──────────────────┬──────────┬──────────┬────────────┤")
    report.append("│ Metric           │ FP32     │ INT8     │ Loss       │")
    report.append("├──────────────────┼──────────┼──────────┼────────────┤")
    report.append(f"│ Accuracy         │ {cls_f32_acc:6.2f} % │ {cls_int8_acc:6.2f} % │ {cls_f32_acc-cls_int8_acc:+5.2f} %    │")
    report.append("└──────────────────┴──────────┴──────────┴────────────┘")
    report.append("")
    report.append("┌─────────────────────────────────────────────────────┐")
    report.append("│  INFERENCE LATENCY (PC simulation — ESP32 will be   │")
    report.append("│  measured on-device with micros())                  │")
    report.append("├──────────────────┬──────────┬──────────┬────────────┤")
    report.append("│ Model            │ FP32     │ INT8     │ Speedup    │")
    report.append("├──────────────────┼──────────┼──────────┼────────────┤")
    report.append(f"│ Classifier       │ {cls_f32_lat:5.3f} ms │ {cls_int8_lat:5.3f} ms │ {cls_f32_lat/max(cls_int8_lat,0.001):5.1f}x      │")
    report.append("└──────────────────┴──────────┴──────────┴────────────┘")
    report.append("")
    report.append("┌─────────────────────────────────────────────────────┐")
    report.append("│  ESP32 MEMORY BUDGET (Estimated)                    │")
    report.append("├──────────────────┬──────────────────────────────────┤")
    report.append(f"│ INT8 Model       │ {cls_int8_size/1024:6.1f} KB (Flash)                │")
    report.append(f"│ Tensor Arena     │   ~4-8 KB (SRAM)                  │")
    report.append(f"│ Feature Buffer   │   ~1.8 KB (SRAM)                  │")
    report.append(f"│ Total SRAM       │  ~10-14 KB / 320 KB = ~4%         │")
    report.append("└──────────────────┴──────────────────────────────────┘")
    report.append("")
    report.append("NOTE: On-device ESP32 latency typically 1-5ms for INT8")
    report.append("      models of this size (30 input features).")
    report.append("")
    report.append("Post-Quantization Classification Report (INT8):")
    report.append(classification_report(y_test, y_pred_int8, target_names=class_names))
    
    report_text = "\n".join(report)
    print(report_text)
    
    # 보고서 파일 저장
    with open("training_report.txt", "w") as f:
        f.write(report_text)
    print("\n✅ Saved: training_report.txt")
    
    # ── Confusion Matrix 시각화 ──
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    for ax, preds, title in [
        (axes[0], y_pred_fp32, "FP32 Classifier"),
        (axes[1], y_pred_int8, "INT8 Quantized Classifier")
    ]:
        cm = confusion_matrix(y_test, preds)
        im = ax.imshow(cm, cmap='Blues')
        ax.set_xticks(range(4))
        ax.set_yticks(range(4))
        ax.set_xticklabels(class_names, rotation=45, ha='right', fontsize=8)
        ax.set_yticklabels(class_names, fontsize=8)
        ax.set_xlabel('Predicted')
        ax.set_ylabel('Actual')
        ax.set_title(title)
        for i in range(4):
            for j in range(4):
                ax.text(j, i, str(cm[i, j]), ha='center', va='center', fontsize=12,
                        color='white' if cm[i, j] > cm.max()/2 else 'black')
    
    plt.tight_layout()
    plt.savefig("confusion_matrix.png", dpi=150, bbox_inches='tight')
    print("✅ Saved: confusion_matrix.png")
    
    print("\n" + "=" * 60)
    print("  📁 Generated files:")
    print("     auv_classifier_int8.h     ← ESP32에 넣을 메인 모델")
    print("     auv_classifier_f32.h      ← 비교용 FP32 모델")
    print("     auv_autoencoder_int8.h    ← 비지도 이상탐지 모델")
    print("     auv_autoencoder_f32.h     ← 비교용")
    print("     auv_norm_params.h         ← 정규화 파라미터")
    print("     training_report.txt       ← 논문에 쓸 수치들")
    print("     confusion_matrix.png      ← 논문 Figure")
    print("=" * 60)


if __name__ == "__main__":
    main()
