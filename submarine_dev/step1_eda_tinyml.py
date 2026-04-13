import pandas as pd
import numpy as np
import matplotlib

matplotlib.use("Agg")  # 백그라운드에서 이미지만 렌더링하고 창 띄우지는 않음
import matplotlib.pyplot as plt
import seaborn as sns
import os

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 1: InWaterSense 데이터셋 EDA 및 전처리
# ==============================================================================
# 이 스크립트는 Jupyter Notebook의 셀(Cell) 단위처럼 주석으로 구분되어 있습니다.
# 각 블록(%%) 단위로 코드를 읽고 필요에 따라 콘솔에서 실행해 볼 수 있습니다.
# ==============================================================================

# %% [Cell 1] 설정 및 데이터 로드
print("\n[Step 1] 데이터 로드 중...")

# InWaterSense 데이터셋 경로 (사용자 환경에 맞게 수정!)
# 현재 찾은 경로: /home/ty/Arduino/sensor_test/ml-data/Static Sensors Data/INWSStaticDataGroupedByTimestamp.csv
data_path = "./ml-data/Static Sensors Data/INWSStaticDataGroupedByTimestamp.csv"

if not os.path.exists(data_path):
    print(f"⚠️ 에러: 파일을 찾을 수 없습니다 -> {data_path}")
    print("경로를 알맞게 수정해 주세요.")
    exit(1)

# 데이터 로드 (결측치가 문자열 'NULL'로 되어 있는 것을 처리)
df = pd.read_csv(data_path, na_values=["", "NA", "NULL", "null"])
print(f"✅ 데이터 로드 완료! Data Shape: {df.shape}")
print(df.head())


# %% [Cell 2] 컬럼명 정리 및 Time Series 설정
print("\n[Step 2] 시계열 설정 및 결측치 확인...")

# 컬럼명을 프로그래밍하기 쉽게 변경 (InWaterSense 원본 컬럼명)
df.rename(
    columns={
        "Timestamp as DateTime": "datetime",
        "Temperature": "temperature",
        "Conductivity": "ec",
        "pH": "ph",
        "DissolvedOxygen": "do",
    },
    inplace=True,
)

# Datetime 변환 (끝부분의 0000, 4000 등 오프셋 잘라내기)
df["datetime"] = df["datetime"].apply(lambda x: " ".join(str(x).split()[:2]))
df["datetime"] = pd.to_datetime(df["datetime"])
df.set_index("datetime", inplace=True)
df.sort_index(inplace=True)

# 필수 센서 컬럼만 추출 (Sensor Channels)
sensor_cols = ["temperature", "ec", "ph", "do"]
df = df[sensor_cols]

# 결측치 확인
print("📊 결측치(NaN) 개수:")
print(df.isnull().sum())

# 간단한 전처리: 선형 보간(Linear Interpolation)으로 결측치 채우기
df.interpolate(method="time", inplace=True)
df.dropna(inplace=True)  # 보간할 수 없는 맨 앞/뒤 결측치 제거
print(f"✔️ 결측치 처리 후 Data Shape: {df.shape}")


# %% [Cell 3] 가상 채널 (Depth, O2, IMU) 및 30개 특징(Features)을 위한 준비
print("\n[Step 3] AUV/ESP32 프로토콜을 위한 가상 채널 주입...")

# InWaterSense에는 Depth, O2, IMU가 없으므로 0.0으로 강제 주입
df["depth"] = 0.0  # 표면 데이터이므로 0m
df["o2"] = 0.0  # I2C 산소센서 데이터 부재
df["imu_energy"] = 0.0  # 모션/IMU 없음

# 전체 채널 확인
all_channels = ["ph", "ec", "do", "o2", "depth", "temperature", "imu_energy"]
print(f"채널 목록: {all_channels}")


# %% [Cell 4] 기초 통계(Describe) 확인
print("\n[Step 4] 데이터 기초 통계 (EDA)...")
print(df[sensor_cols].describe())


# %% [Cell 5] 시계열 그래프 시각화 (Time Series Plot)
def plot_timeseries(dataframe):
    print("\n[Step 5] 시계열 그래프를 렌더링합니다... (창을 닫으면 다음으로 진행)")
    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(14, 10), sharex=True)
    fig.suptitle("InWaterSense Sensor Data Time Series", fontsize=16)

    sns.lineplot(
        data=dataframe, x=dataframe.index, y="temperature", ax=axes[0], color="red"
    )
    axes[0].set_ylabel("Temperature (°C)")

    sns.lineplot(data=dataframe, x=dataframe.index, y="ec", ax=axes[1], color="orange")
    axes[1].set_ylabel("Conductivity (µS/cm)")

    sns.lineplot(data=dataframe, x=dataframe.index, y="ph", ax=axes[2], color="green")
    axes[2].set_ylabel("pH")

    sns.lineplot(data=dataframe, x=dataframe.index, y="do", ax=axes[3], color="blue")
    axes[3].set_ylabel("DO (mg/L or %)")

    plt.tight_layout()
    plt.savefig("timeseries_plot.png")  # 파일로도 저장
    plt.show()


plot_timeseries(df)


# %% [Cell 6] 이상치 탐지를 위한 Boxplot
def plot_boxplots(dataframe):
    print("\n[Step 6] 이상치 파악을 위한 Boxplot 렌더링... (창을 닫으면 종료)")
    fig, axes = plt.subplots(nrows=1, ncols=4, figsize=(14, 5))
    fig.suptitle("Outlier Detection (Boxplot)", fontsize=16)

    sns.boxplot(data=dataframe, y="temperature", ax=axes[0], color="red")
    sns.boxplot(data=dataframe, y="ec", ax=axes[1], color="orange")
    sns.boxplot(data=dataframe, y="ph", ax=axes[2], color="green")
    sns.boxplot(data=dataframe, y="do", ax=axes[3], color="blue")

    plt.tight_layout()
    plt.savefig("boxplots.png")
    plt.show()


plot_boxplots(df)

print("\n🎉 Step 1 EDA 완료! (timeseries_plot.png, boxplots.png 저장됨)")
print("다음 Step에서는 이 DataFrame을 64개 단위 Window로 자르고(Striding),")
print("정의하신 수학 헬퍼(calcMean, calcStd, calcCorrelation 등)를 사용하여")
print("30개의 특징 벡터(Feature Vector) 배열로 변환하는 작업을 진행해야 합니다.")
