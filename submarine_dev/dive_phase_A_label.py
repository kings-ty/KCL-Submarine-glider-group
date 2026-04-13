import pandas as pd
import numpy as np
import os
import glob
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ==============================================================================
# 🌊 Dive Phase A: AUV_navigation_dataset → Dive Phase 자동 라벨 생성
#
# 라벨 정의:
#   0 = SURFACE   : 수면, |pitch| 작음, 이동 거의 없음
#   1 = DIVING    : pitch 음수 (nose down), 하강 중
#   2 = CRUISE    : 순항 (바닥 근처 or 일정 고도 유지, |pitch| 작음)
#   3 = ASCENDING : pitch 양수 (nose up), 상승 중
#
# 핵심 지표: pitch (1차), vu 수직속도 (2차), alt 고도변화율 (3차)
# 출력: dive_phase_labels.csv
# ==============================================================================

DATASET_DIR = "./AUV_navigation_dataset"
OUTPUT_CSV = "./dive_phase_labels.csv"
OUTPUT_PLOT = "./ml-data/dive_phase_overview.png"

os.makedirs("./ml-data", exist_ok=True)

# ─────────────────────────────────────────────
# [Cell 1] 모든 CSV 파일 수집
# ─────────────────────────────────────────────
print("=" * 60)
print("🌊 Dive Phase A: 자동 라벨 생성")
print("=" * 60)

csv_files = sorted(glob.glob(f"{DATASET_DIR}/**/*.csv", recursive=True))
# .git 폴더 제외
csv_files = [f for f in csv_files if ".git" not in f]
print(f"\n✅ CSV 파일 {len(csv_files)}개 발견:")
for f in csv_files:
    print(f"   {os.path.relpath(f, DATASET_DIR)}")


# ─────────────────────────────────────────────
# [Cell 2] 컬럼 정규화 함수
# 파일마다 lat_gps vs lat 등 컬럼명이 다름
# ─────────────────────────────────────────────
def normalize_columns(df):
    """컬럼명 통일: lat_gps→lat, lon_gps→lon, alt_gps→alt 등"""
    rename_map = {
        "lat_gps": "lat",
        "lon_gps": "lon",
        "alt_gps": "alt",
        "time_gps": "time_gps_extra",
    }
    df = df.rename(columns={k: v for k, v in rename_map.items() if k in df.columns})

    # height 컬럼이 있으면 alt 보조로 사용
    if "alt" not in df.columns and "height" in df.columns:
        df["alt"] = df["height"]
    if "alt" not in df.columns:
        df["alt"] = 0.0

    # depth가 없으면 0
    if "depth" not in df.columns:
        df["depth"] = 0.0

    # 반드시 필요한 컬럼이 없으면 0으로 채움
    required = [
        "roll",
        "pitch",
        "yaw",
        "vf",
        "vl",
        "vu",
        "ax",
        "ay",
        "az",
        "wx",
        "wy",
        "wz",
        "time",
    ]
    for col in required:
        if col not in df.columns:
            df[col] = 0.0

    return df


# ─────────────────────────────────────────────
# [Cell 3] State Machine 라벨러
# pitch (rad) 기반 + vu (수직속도) 보조
#
# 히스테리시스 설계: 상태 전환이 잦지 않도록 min_hold=10 스텝 유지
# ─────────────────────────────────────────────

# pitch 임계값 (radian)
PITCH_DIVE_TH = -0.15  # ~-8.6° → 하강 판정
PITCH_ASCEND_TH = 0.15  # ~+8.6° → 상승 판정
VU_TH = 0.05  # vu 절대값 > 이걸 넘으면 수직 운동 중
MIN_HOLD = 10  # 최소 동일 상태 유지 스텝 수 (노이즈 억제)

# 라벨 상수
SURFACE = 0
DIVING = 1
CRUISE = 2
ASCENDING = 3
LABEL_NAMES = {0: "SURFACE", 1: "DIVING", 2: "CRUISE", 3: "ASCENDING"}


def state_machine_label(df):
    """
    pitch + vu 기반 State Machine으로 각 타임스텝에 Dive Phase 라벨 부여
    """
    pitch = df["pitch"].values
    vu = df["vu"].values
    alt = df["alt"].values

    n = len(df)
    labels = np.zeros(n, dtype=np.int32)

    state = SURFACE  # 초기 상태
    hold_counter = 0  # 현재 상태 유지 스텝 수

    for i in range(n):
        p = pitch[i]
        v = vu[i]
        da = (alt[i] - alt[i - 1]) if i > 0 else 0.0  # alt 변화율

        # 상태 전환 후보 결정
        if p < PITCH_DIVE_TH:
            candidate = DIVING
        elif p > PITCH_ASCEND_TH:
            candidate = ASCENDING
        else:
            # pitch 작으면 수직속도로 보조 판단
            if v < -VU_TH or da < -0.05:  # 내려가는 중
                candidate = DIVING
            elif v > VU_TH or da > 0.05:  # 올라가는 중
                candidate = ASCENDING
            else:
                # 정지에 가까움 → alt 가 낮은지로 SURFACE vs CRUISE 구분
                # alt 데이터 전체 중간값보다 낮으면 CRUISE(바닥 근처), 높으면 SURFACE
                candidate = CRUISE

        # 히스테리시스: 새 후보가 현재 상태와 다를 때만 hold_counter 초기화
        if candidate != state:
            hold_counter += 1
            if hold_counter >= MIN_HOLD:
                state = candidate
                hold_counter = 0
        else:
            hold_counter = 0

        labels[i] = state

    # SURFACE vs CRUISE 후처리:
    # alt 중앙값을 기준으로 alt가 높은 구간 → SURFACE, 낮은 구간 → CRUISE
    alt_median = np.median(alt[alt != 0]) if np.any(alt != 0) else 0.0
    for i in range(n):
        if labels[i] == CRUISE and alt[i] > alt_median * 0.8:
            labels[i] = SURFACE

    return labels


# ─────────────────────────────────────────────
# [Cell 4] 전체 파일 처리 및 라벨 생성
# ─────────────────────────────────────────────
print("\n[Step 1] 전체 CSV 파일 라벨링 수행...")

all_records = []  # (file_id, row_idx, label, pitch, vu, alt, time) 저장

for file_idx, fpath in enumerate(csv_files):
    try:
        df = pd.read_csv(fpath)
        df = normalize_columns(df)

        labels = state_machine_label(df)

        for i, lbl in enumerate(labels):
            all_records.append(
                {
                    "file_id": file_idx,
                    "file_name": os.path.relpath(fpath, DATASET_DIR),
                    "row_idx": i,
                    "time": df["time"].iloc[i] if "time" in df.columns else i,
                    "label": lbl,
                    "label_name": LABEL_NAMES[lbl],
                    "pitch": df["pitch"].iloc[i],
                    "vu": df["vu"].iloc[i],
                    "alt": df["alt"].iloc[i],
                }
            )

        # 파일별 라벨 분포 출력
        unique, counts = np.unique(labels, return_counts=True)
        dist = {LABEL_NAMES[u]: c for u, c in zip(unique, counts)}
        print(
            f"   [{file_idx:02d}] {os.path.basename(fpath):<30} "
            f"총 {len(labels)}행 | {dist}"
        )

    except Exception as e:
        print(f"   ⚠️  [{file_idx}] {fpath} 에러: {e}")
        continue

# ─────────────────────────────────────────────
# [Cell 5] 결과 저장
# ─────────────────────────────────────────────
df_labels = pd.DataFrame(all_records)
df_labels.to_csv(OUTPUT_CSV, index=False)
print(f"\n✅ dive_phase_labels.csv 저장 완료: {OUTPUT_CSV}")
print(f"   총 {len(df_labels):,}행 | 파일 {df_labels['file_id'].nunique()}개")

# 전체 라벨 분포
print(f"\n📊 전체 라벨 분포:")
total = len(df_labels)
for lbl_id, lbl_name in LABEL_NAMES.items():
    cnt = (df_labels["label"] == lbl_id).sum()
    print(f"   [{lbl_id}] {lbl_name:<15}: {cnt:>7,}행 ({100 * cnt / total:.1f}%)")

# ─────────────────────────────────────────────
# [Cell 6] 시각화: 최대 3개 파일 pitch + label 플롯
# ─────────────────────────────────────────────
print("\n[Step 2] 라벨 시각화 저장 중...")

COLORS = {0: "#2ecc71", 1: "#e74c3c", 2: "#3498db", 3: "#f39c12"}
n_plot = min(3, len(csv_files))
fig, axes = plt.subplots(n_plot, 1, figsize=(14, 4 * n_plot))
if n_plot == 1:
    axes = [axes]

for plot_i, fpath in enumerate(csv_files[:n_plot]):
    df_f = df_labels[df_labels["file_name"] == os.path.relpath(fpath, DATASET_DIR)]
    labels = df_f["label"].values
    pitch = df_f["pitch"].values
    t = np.arange(len(labels))

    ax = axes[plot_i]
    # 배경 컬러 (Dive Phase)
    for j in range(len(t)):
        ax.axvspan(t[j], t[j] + 1, color=COLORS[labels[j]], alpha=0.35, linewidth=0)
    ax.plot(t, np.degrees(pitch), "k-", linewidth=0.8, label="Pitch (deg)")
    ax.axhline(0, color="gray", linewidth=0.5, linestyle="--")
    ax.set_title(f"{os.path.basename(fpath)} — Dive Phase Labels", fontsize=11)
    ax.set_ylabel("Pitch (°)")
    ax.set_xlabel("Sample Index")
    if plot_i == 0:
        for lbl_id, lbl_name in LABEL_NAMES.items():
            ax.fill_between([], [], color=COLORS[lbl_id], alpha=0.5, label=lbl_name)
        ax.legend(loc="upper right", fontsize=8)

plt.tight_layout()
plt.savefig(OUTPUT_PLOT, dpi=150, bbox_inches="tight")
print(f"✔️ 시각화 저장: {OUTPUT_PLOT}")

print("\n🎉 Step A 완료! dive_phase_labels.csv 준비됨.")
print("   → 다음: python dive_phase_B_train.py")
