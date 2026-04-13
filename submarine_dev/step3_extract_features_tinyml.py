import pandas as pd
import numpy as np
import os

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 3: 슬라이딩 윈도우 특징 추출 (Feature Extraction)
# ==============================================================================

# %% [Cell 1] 설정 및 데이터 로드
print("\n[Step 1] Step 2에서 전처리된 데이터 로드 중...")
data_path = "./ml-data/step2_preprocessed_data.csv"
if not os.path.exists(data_path):
    print(f"⚠️ 에러: 전처리된 파일이 없습니다 -> {data_path}")
    exit(1)

df = pd.read_csv(data_path, index_col="datetime", parse_dates=True)
print(f"✅ 로드 완료! Shape: {df.shape}")

# 채널 (C++ 인덱스 순서와 동일)
# 0: ph, 1: ec, 2: do, 3: o2, 4: depth, 5: temperature, 6: imu_energy
data_array = df[["ph", "ec", "do", "o2", "depth", "temperature", "imu_energy"]].values

WINDOW_SIZE = 64
STRIDE = 16
NUM_FEATURES = 30


# Numpy 배열을 이용한 효율적인 슬라이딩 윈도우 생성 함수
def create_windows(data, window_size, stride):
    num_windows = (len(data) - window_size) // stride + 1
    windows = []
    for i in range(0, (num_windows) * stride, stride):
        windows.append(data[i : i + window_size])
    return np.array(windows)


# %% [Cell 2] 수학 연산 헬퍼 (C++ sensor_math.h 와 완전히 동일하게 동작)
def calc_slope(y):
    # 선형 회귀 기울기 (x는 인덱스 0~N-1)
    x = np.arange(len(y))
    sumX = np.sum(x)
    sumY = np.sum(y)
    sumXY = np.sum(x * y)
    sumX2 = np.sum(x * x)
    n = len(y)
    denom = n * sumX2 - sumX**2
    if np.abs(denom) < 1e-10:
        return 0.0
    return (n * sumXY - sumX * sumY) / denom


def calc_rms(y):
    return np.sqrt(np.mean(y**2))


def calc_corr(x, y):
    # 피어슨 상관계수
    num = np.sum((x - np.mean(x)) * (y - np.mean(y)))
    denom = np.sqrt(np.sum((x - np.mean(x)) ** 2) * np.sum((y - np.mean(y)) ** 2))
    if denom < 1e-10:
        return 0.0
    return num / denom


# %% [Cell 3] 64윈도우 데이터를 30개 특징으로 변환하는 함수
def extract_features_from_window(window_data):
    # window_data shape: (64, 7)
    ph = window_data[:, 0]
    ec = window_data[:, 1]
    do = window_data[:, 2]
    o2 = window_data[:, 3]
    depth = window_data[:, 4]
    temp = window_data[:, 5]
    imu = window_data[:, 6]

    fv = np.zeros(NUM_FEATURES)

    # [1~4] pH: mean, std, slope, rms
    fv[0] = np.mean(ph)
    fv[1] = np.std(ph, ddof=0)  # C++ 1/N 계산과 맞춤
    fv[2] = calc_slope(ph)
    fv[3] = calc_rms(ph)

    # [5~8] EC
    fv[4] = np.mean(ec)
    fv[5] = np.std(ec, ddof=0)
    fv[6] = calc_slope(ec)
    fv[7] = calc_rms(ec)

    # [9~12] DO
    fv[8] = np.mean(do)
    fv[9] = np.std(do, ddof=0)
    fv[10] = calc_slope(do)
    fv[11] = calc_rms(do)

    # [13~16] O2
    fv[12] = np.mean(o2)
    fv[13] = np.std(o2, ddof=0)
    fv[14] = calc_slope(o2)
    fv[15] = calc_rms(o2)

    # [17~19] Depth: mean, std, slope
    fv[16] = np.mean(depth)
    fv[17] = np.std(depth, ddof=0)
    fv[18] = calc_slope(depth)

    # [20~22] Temp: mean, std, slope
    fv[19] = np.mean(temp)
    fv[20] = np.std(temp, ddof=0)
    fv[21] = calc_slope(temp)

    # [23~25] IMU_Energy: mean, std, max
    fv[22] = np.mean(imu)
    fv[23] = np.std(imu, ddof=0)
    fv[24] = np.max(imu)

    # [26] Cross-Channel: pH-Temp Correlation
    fv[25] = calc_corr(ph, temp)

    # [27~28] pH Residual: actual_pH - expected_pH_from_temp
    expected_ph = 7.0 + (-0.02 * (temp - 20.0))
    ph_res = ph - expected_ph
    fv[26] = np.mean(ph_res)
    fv[27] = np.std(ph_res, ddof=0)

    # [29~30] DO Residual: actual_DO - expected_DO_from_depth_temp
    expected_do = 14.6 - 0.39 * temp + 0.007 * depth
    do_res = do - expected_do
    fv[28] = np.mean(do_res)
    fv[29] = np.std(do_res, ddof=0)

    return fv


# %% [Cell 4] 처리 및 결과 저장
print(f"\n[Step 2] 윈도우 슬라이싱 수행 (Size={WINDOW_SIZE}, Stride={STRIDE})...")
windows = create_windows(data_array, WINDOW_SIZE, STRIDE)
print(f"✔️ 총 생성된 윈도우 개수: {windows.shape[0]} 개")

print(f"\n[Step 3] 30개 특징 추출 (Feature Extraction) 진행...")
features_list = []
for index, win in enumerate(windows):
    features_list.append(extract_features_from_window(win))

X_features = np.array(features_list)
print(f"✔️ X_features 데이터 형태: {X_features.shape} (N_Windows, NUM_FEATURES)")

# Numpy 바이너리로 저장 (용량 최적화, 로딩 속도 추후 편리)
out_npy_path = "./ml-data/X_features.npy"
np.save(out_npy_path, X_features)

print(
    f"\n🎉 Step 3 특징 추출 완료! 결과가 {out_npy_path} 에 Numpy 배열 구조로 매우 완벽하게 저장되었습니다."
)
print(
    "이 배열 데이터는 C++ 내부 `float[30]` 버퍼 구조와 1:1로 전부 매핑되는 구조입니다."
)
