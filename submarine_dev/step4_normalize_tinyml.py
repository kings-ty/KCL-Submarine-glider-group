import numpy as np
import os

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 4: Min-Max 정규화 및 ESP32용 파라미터 추출
# ==============================================================================

# %% [Cell 1] 특징 벡터(Feature Vector) 데이터 로드
print("\n[Step 1] X_features.npy 로드 중...")
data_path = './ml-data/X_features.npy'
if not os.path.exists(data_path):
    print("⚠️ 에러: 특징 벡터 파일을 찾을 수 없습니다!")
    exit(1)

X = np.load(data_path)
print(f"✅ 데이터 로드 완료! Shape: {X.shape}")

# %% [Cell 2] Min-Max 파라미터 (0 ~ 1 스케일링) 계산
print("\n[Step 2] Min-Max Normalization 스케일 계산...")
X_min = np.min(X, axis=0)
X_max = np.max(X, axis=0)
X_range = X_max - X_min

# 센서 변동이 아예 없어서 최대-최소 차이가 0일 경우, 0으로 나누어지는 오류를 막기 위해 1.0으로 보정
X_range[X_range == 0] = 1.0

# 정규화 수식: (X - min) / range
X_scaled = (X - X_min) / X_range

# 다음 훈련(Step 5) 단계에서 사용할 수 있도록 정규화된 Numpy 파일을 백업 저장
out_npy_path = './ml-data/X_features_scaled.npy'
np.save(out_npy_path, X_scaled)
print(f"✔️ 정규화 완료! 스케일링된 데이터가 저장되었습니다: {out_npy_path}")

# Step 8 (양자화) 에서 .npy로 불러올 수 있도록 mins/ranges 별도 저장
np.save('./ml-data/feature_mins.npy',   X_min)
np.save('./ml-data/feature_ranges.npy', X_range)
print("✔️ 정규화 파라미터 저장: feature_mins.npy / feature_ranges.npy")


# %% [Cell 3] C++ / ESP32용 Float 배열 추출
print("\n[Step 3] C++ 모델 탑재를 위한 C 헤더 자동 생성...")

def format_c_array(name, arr):
    # 가독성을 위해 5개 단위로만 줄바꿈을 하여 C 코드를 생성합니다.
    items = [f"{val:.6f}f" for val in arr]
    body = ",\n    ".join([", ".join(items[i:i+5]) for i in range(0, len(items), 5)])
    return f"const float {name}[30] = {{\n    {body}\n}};\n"

c_header_path = './ml-data/normalization_arrays.h'
with open(c_header_path, 'w', encoding='utf-8') as f:
    f.write("// =======================================================\n")
    f.write("// 🎯 특징 벡터(30차원) Min-Max 정규화 상수 (TinyML 탑재용)\n")
    f.write("// \n")
    f.write("// 작동 원리: ESP32에서 추출한 원본 30개의 Feature 값들을\n")
    f.write("// 학습된 ML 모델에 넣기 전에 동일하게 0~1 비율로 압축해야 합니다.\n")
    f.write("// \n")
    f.write("// [ESP32 코드 사용 예시]:\n")
    f.write("// for (int i = 0; i < 30; i++) {\n")
    f.write("//    currentFeaturesArray[i] = (currentFeaturesArray[i] - feature_mins[i]) / feature_ranges[i];\n")
    f.write("// }\n")
    f.write("// =======================================================\n\n")
    
    f.write(format_c_array("feature_mins", X_min))
    f.write("\n")
    f.write(format_c_array("feature_ranges", X_range))

print(f"\n🎉 Step 4 완료! ESP32 펌웨어에 그대로 복사해 넣을 수 있는 C 상수배열 코드가 {c_header_path} 에 저장되었습니다.")
