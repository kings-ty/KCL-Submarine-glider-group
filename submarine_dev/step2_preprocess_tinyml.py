import pandas as pd
import numpy as np
import os

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 2: InWaterSense 데이터셋 전처리
# ==============================================================================

# %% [Cell 1] 원본 데이터 로드 및 시계열 처리
print("\n[Step 1] 원본 데이터 로드 중...")
data_path = "./ml-data/Static Sensors Data/INWSStaticDataGroupedByTimestamp.csv"
if not os.path.exists(data_path):
    print(f"⚠️ 에러: 파일을 찾을 수 없습니다 -> {data_path}")
    exit(1)

df = pd.read_csv(data_path, na_values=["", "NA", "NULL", "null"])

# 컬럼명 정리
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

# Datetime 변환 (오프셋 무시)
df["datetime"] = df["datetime"].apply(lambda x: " ".join(str(x).split()[:2]))
df["datetime"] = pd.to_datetime(df["datetime"])
df.set_index("datetime", inplace=True)
df.sort_index(inplace=True)

# 필수 센서 컬럼 추출
sensor_cols = ["temperature", "ec", "ph", "do"]
df = df[sensor_cols]

print(f"✅ 데이터 로드 완료! Shape: {df.shape}")


# %% [Cell 2] 지시사항 1: 결측치 처리 (Forward Fill)
print("\n[Step 2] 결측치 처리 (Forward Fill)...")
df.ffill(inplace=True)
# 가장 첫 번째 행이 결측치일 경우 앞쪽도 마저 채우기(bfill) (안전 장치)
df.bfill(inplace=True)
print(f"✔️ 결측치 처리 후 Shape: {df.shape}")


# %% [Cell 3] 지시사항 2: 물리적 결함(불가능한 값) 범위를 고장 플래그로 마킹 (삭제하지 않음)
print("\n[Step 3] 물리적으로 불가능한 범위의 데이터 센서 고장 마킹...")
VALID_RANGES = {
    "temperature": (-5, 60),    # 수온의 물리적 한계
    "ec": (-100, float('inf')), # -100 미만 오류 (상한은 정상적인 오염 상태 모두 포함)
    "ph": (0, 14),              # pH 물리적 한계
    "do": (0, float('inf')),    # DO 음수 오류 (상한은 정상적인 과포화 포함)
}

# 2단계: 범위 밖 데이터를 "고장 플래그"로 마킹 (삭제하지 않음!)
df["is_sensor_fault"] = False

for col, (vmin, vmax) in VALID_RANGES.items():
    mask = (df[col] < vmin) | (df[col] > vmax)
    df.loc[mask, "is_sensor_fault"] = True
    fault_count = mask.sum()
    print(f"  {col}: {fault_count} samples outside [{vmin}, {vmax}]")

total_faults = df["is_sensor_fault"].sum()
print(
    f"\n  Total sensor fault samples: {total_faults} / {len(df)} "
    f"({total_faults / len(df) * 100:.1f}%)"
)


# %% [Cell 4] 지시사항 3: 가상 채널(depth, o2, imu_energy) 추가
print("\n[Step 4] 미지원 가상 센서 채널(0.0) 생성 및 순서 정렬...")
df["depth"] = 0.0
df["o2"] = 0.0
df["imu_energy"] = 0.0

# 채널 순서를 C++ 헤더(sensor_types.h)에 정의한 ChannelIndexenum 순서와 일치시킵니다.
# 결과 분석을 위해 is_sensor_fault 플래그도 맨 뒤에 포함합니다.
final_cols = [
    "ph",
    "ec",
    "do",
    "o2",
    "depth",
    "temperature",
    "imu_energy",
    "is_sensor_fault",
]
df = df[final_cols]

print(f"✔️ C++ 데이터 통신 레이어와 동일하게 채널 병합 완료:")
print(df.head())


# %% [Cell 5] 결과 저장
output_path = "./ml-data/step2_preprocessed_data.csv"
df.to_csv(output_path)
print(f"\n🎉 Step 2 전처리 완료! 결과가 {output_path} 에 저장되었습니다.")
print(
    "이제 다음 Step에서 슬라이딩 윈도우(크기:64, 스트라이드:16)를 적용시켜 30개의 Feature로 압축할 준비가 끝났습니다!"
)
