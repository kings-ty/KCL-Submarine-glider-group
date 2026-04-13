import numpy as np
import pandas as pd
import os
import json
import zipfile
import shutil

# ==============================================================================
# 🌊 [TinyML/Edge AI] Step 8 (대안): Edge Impulse Studio 형식으로 내보내기
#
# Edge Impulse 가져오기 방식 3가지 모두 지원:
#
#   [방법 A] CSV Wizard (권장)
#     → ei_combined.csv : label + 30개 특징 한 파일
#     → Edge Impulse Studio > Data acquisition > Upload > CSV Wizard
#
#   [방법 B] 개별 CSV (라벨별 파일 분리)
#     → ei_samples/NORMAL.sample0.csv, SENSOR_FAULT.sample1.csv, ...
#     → 각 파일명 앞에 라벨명 붙이면 자동 라벨링됨
#
#   [방법 C] Edge Impulse CLI uploader용 (features.csv + labels.csv 분리)
#     → ei_features.csv, ei_labels.csv
#     → edge-impulse-uploader --format csv 에 사용
# ==============================================================================

print("=" * 60)
print("🌊 Edge Impulse Studio 데이터 내보내기 시작")
print("=" * 60)

# ─────────────────────────────────────────────
# [Cell 1] 데이터 로드
# ─────────────────────────────────────────────
print("\n[Step 1] Step 3/4/6 결과 파일 로드 중...")

X_raw    = np.load('./ml-data/X_features.npy')        # 정규화 전 원시 특징 (실제값)
X_scaled = np.load('./ml-data/X_features_scaled.npy') # 0~1 정규화된 특징
y_labels = np.load('./ml-data/y_labels.npy')          # 자동 라벨 (0~3)

if not os.path.exists('./ml-data/X_features.npy'):
    print("⚠️ X_features.npy 없음! Step 3을 먼저 실행하세요.")
    exit(1)
if not os.path.exists('./ml-data/y_labels.npy'):
    print("⚠️ y_labels.npy 없음! Step 6을 먼저 실행하세요.")
    exit(1)

# 클래스 이름 정의 (Edge Impulse는 label을 문자열로 사용)
CLASS_NAMES = {
    0: 'NORMAL',
    1: 'MOTION_ARTIFACT',
    2: 'SENSOR_FAULT',
    3: 'ENV_ANOMALY'
}

# 30개 특징 이름 (Edge Impulse 헤더에 사용)
FEATURE_NAMES = [
    'pH_mean',    'pH_std',    'pH_slope',    'pH_rms',
    'EC_mean',    'EC_std',    'EC_slope',    'EC_rms',
    'DO_mean',    'DO_std',    'DO_slope',    'DO_rms',
    'O2_mean',    'O2_std',    'O2_slope',    'O2_rms',
    'depth_mean', 'depth_std', 'depth_slope',
    'temp_mean',  'temp_std',  'temp_slope',
    'imu_mean',   'imu_std',   'imu_max',
    'pH_temp_corr',
    'pH_res_mean', 'pH_res_std',
    'DO_res_mean', 'DO_res_std'
]

label_strings = np.array([CLASS_NAMES[i] for i in y_labels])

N = len(X_raw)
print(f"✅ 로드 완료: {N}개 윈도우, 30개 특징")
print(f"\n   라벨 분포:")
for k, v in CLASS_NAMES.items():
    cnt = (y_labels == k).sum()
    print(f"   [{k}] {v:<20}: {cnt:>5}개 ({100*cnt/N:.1f}%)")

# ─────────────────────────────────────────────
# [Cell 2] 출력 디렉토리 준비
# ─────────────────────────────────────────────
os.makedirs('./ei_export', exist_ok=True)
os.makedirs('./ei_export/individual_samples', exist_ok=True)

# ─────────────────────────────────────────────
# [Cell 3] 방법 A: CSV Wizard용 단일 CSV
# 형식: label, feature_0, feature_1, ..., feature_29
# Edge Impulse Studio > Data acquisition > Upload data
#   > "Upload into category" = Split automatically
#   > Format = CSV, 첫 열 = label
# ─────────────────────────────────────────────
print("\n[Step 2] 방법 A: CSV Wizard용 단일 통합 CSV 생성...")

# 정규화된 값 사용 (0~1 범위가 Edge Impulse DSP 처리에 더 안정적)
df_combined = pd.DataFrame(X_scaled, columns=FEATURE_NAMES)
df_combined.insert(0, 'label', label_strings)  # 첫 번째 열에 라벨 삽입

combined_path = './ei_export/ei_combined.csv'
df_combined.to_csv(combined_path, index=False, float_format='%.6f')
print(f"✔️ {combined_path} 저장 완료  ({os.path.getsize(combined_path)//1024} KB)")
print(f"   → Shape: {df_combined.shape}  (행=샘플, 열=label+30특징)")

# ─────────────────────────────────────────────
# [Cell 4] 방법 B: 라벨별 개별 CSV (파일명 라벨 자동인식)
# 형식: feature_0, feature_1, ..., feature_29 (헤더 있음, 라벨 없음)
# 파일명: LABEL_NAME.sampleN.csv
# Edge Impulse가 파일명 앞부분을 라벨로 자동 인식함
# ─────────────────────────────────────────────
print("\n[Step 3] 방법 B: 라벨별 개별 CSV 파일 생성...")

df_features_only = pd.DataFrame(X_scaled, columns=FEATURE_NAMES)

sample_counters = {name: 0 for name in CLASS_NAMES.values()}
file_count = 0

for idx in range(N):
    label_name = CLASS_NAMES[y_labels[idx]]
    sample_num = sample_counters[label_name]
    filename   = f"{label_name}.sample{sample_num:04d}.csv"
    filepath   = f"./ei_export/individual_samples/{filename}"

    # 단일 행 CSV (헤더 포함)
    df_features_only.iloc[[idx]].to_csv(filepath, index=False, float_format='%.6f')

    sample_counters[label_name] += 1
    file_count += 1

    # 진행 상황 로그 (100개마다)
    if file_count % 500 == 0:
        print(f"   진행 중: {file_count}/{N} 파일 생성...")

print(f"✔️ 개별 CSV 생성 완료: {file_count}개 파일")
print(f"   저장 위치: ./ei_export/individual_samples/")
for name, cnt in sample_counters.items():
    print(f"   {name}: {cnt}개 파일")

# ─────────────────────────────────────────────
# [Cell 5] 방법 C: edge-impulse-uploader CLI용
# features.csv (헤더 + 특징값만)
# labels.csv   (헤더 없이 라벨 문자열만, 같은 행 순서)
# ─────────────────────────────────────────────
print("\n[Step 4] 방법 C: CLI 업로더용 features.csv + labels.csv 생성...")

# features.csv: 헤더 포함, 30개 특징
features_path = './ei_export/ei_features.csv'
df_features_only.to_csv(features_path, index=False, float_format='%.6f')
print(f"✔️ {features_path}  ({os.path.getsize(features_path)//1024} KB)")

# labels.csv: 헤더 없이 라벨 문자열만
labels_path = './ei_export/ei_labels.csv'
pd.Series(label_strings).to_csv(labels_path, index=False, header=False)
print(f"✔️ {labels_path}  ({os.path.getsize(labels_path)//1024} KB)")

# 행 수 일치 검증
n_features = sum(1 for _ in open(features_path)) - 1  # 헤더 제외
n_labels   = sum(1 for _ in open(labels_path))
print(f"\n   ✔️ 행 수 일치 검증: features={n_features}행, labels={n_labels}행", end="")
print("  ✅ OK" if n_features == n_labels else "  ❌ 불일치!")

# ─────────────────────────────────────────────
# [Cell 6] 원시값(Raw) 버전도 함께 저장
# (Edge Impulse에서 DSP 블록 직접 설계할 때 원시값이 필요한 경우)
# ─────────────────────────────────────────────
print("\n[Step 5] 원시(Raw) 특징값 버전 추가 저장...")

df_raw_combined = pd.DataFrame(X_raw, columns=FEATURE_NAMES)
df_raw_combined.insert(0, 'label', label_strings)
raw_path = './ei_export/ei_combined_raw.csv'
df_raw_combined.to_csv(raw_path, index=False, float_format='%.6f')
print(f"✔️ {raw_path} → 정규화 전 원시값 (Edge Impulse 자체 Normalization 쓸 때)")

# ─────────────────────────────────────────────
# [Cell 7] README 생성 (Edge Impulse 업로드 방법 안내)
# ─────────────────────────────────────────────
readme_content = """# Edge Impulse Export - AUV Anomaly Detection
Dataset: InWaterSense (Ahmedi 2021) | Classes: 4 | Features: 30 | Windows: {N}

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
방법 A: CSV Wizard (가장 쉬운 방법)  ★ 권장
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
파일: ei_combined.csv

1. Edge Impulse Studio 접속 (studio.edgeimpulse.com)
2. 프로젝트 생성 또는 열기
3. [Data acquisition] 탭 클릭
4. [Upload data] 버튼 클릭
5. ei_combined.csv 파일 선택
6. "Upload into category" = "Split 80/20 (training/testing) automatically"
7. "File format" = "CSV file"
8. [Begin upload] 클릭
9. CSV Wizard 팝업:
   - "Label column" = "label"
   - 나머지 열 = features (자동 감지)
   - [Done] 클릭
10. Data acquisition 화면에서 4개 클래스 확인

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
방법 B: 개별 CSV 파일 (파일명 자동 라벨링)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
폴더: individual_samples/

1. Edge Impulse Studio > [Data acquisition] > [Upload data]
2. 파일 선택: individual_samples 폴더 내 전체 파일 선택
3. "Label" = "Infer from filename" 선택
   (파일명의 첫 부분 NORMAL, SENSOR_FAULT 등이 자동 라벨됨)
4. [Begin upload] 클릭

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
방법 C: Edge Impulse CLI uploader
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
파일: ei_features.csv, ei_labels.csv

# CLI 설치
npm install -g edge-impulse-cli

# 로그인
edge-impulse-login

# 업로드 (프로젝트 이름 지정)
edge-impulse-uploader --format csv \\
  --label-path ./ei_labels.csv \\
  ./ei_features.csv

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
파일 설명
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
ei_combined.csv         → label + 30특징 (0~1 정규화) [방법 A]
ei_combined_raw.csv     → label + 30특징 (원시값, EI 자체 정규화용)
ei_features.csv         → 30특징만 (0~1 정규화) [방법 C]
ei_labels.csv           → 라벨 문자열만 [방법 C]
individual_samples/     → 라벨별 개별 파일 [방법 B]

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
클래스 분포
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
{class_dist}
""".format(
    N=N,
    class_dist="\n".join([
        f"  {CLASS_NAMES[k]:<20}: {(y_labels==k).sum():>5}개 ({100*(y_labels==k).sum()/N:.1f}%)"
        for k in range(4)
    ])
)

with open('./ei_export/IMPORT_GUIDE.md', 'w') as f:
    f.write(readme_content)
print("✔️ ./ei_export/IMPORT_GUIDE.md 가이드 파일 저장 완료")

# ─────────────────────────────────────────────
# [Cell 8] ZIP으로 압축 (업로드 편의성)
# ─────────────────────────────────────────────
print("\n[Step 6] ei_export.zip 압축 생성 중...")

zip_path = './ei_export.zip'
with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
    for root, dirs, files in os.walk('./ei_export'):
        # 개별 샘플 파일은 수천 개라 압축 오래 걸림 → 별도 zip
        if 'individual_samples' in root:
            continue
        for file in files:
            full_path = os.path.join(root, file)
            arc_name  = os.path.relpath(full_path, './ei_export')
            zf.write(full_path, arc_name)

# 개별 샘플은 별도 zip
zip_individual_path = './ei_export_individual.zip'
with zipfile.ZipFile(zip_individual_path, 'w', zipfile.ZIP_DEFLATED) as zf:
    for root, dirs, files in os.walk('./ei_export/individual_samples'):
        for file in files:
            full_path = os.path.join(root, file)
            zf.write(full_path, file)  # 폴더 없이 flat 구조

print(f"✔️ {zip_path} → CSV Wizard + CLI용 파일 묶음")
print(f"✔️ {zip_individual_path} → 개별 CSV 파일 묶음")

# ─────────────────────────────────────────────
# [Cell 9] 최종 요약 출력
# ─────────────────────────────────────────────
print("\n" + "="*60)
print("🎉 Edge Impulse Export 완료!")
print("="*60)

output_files = [
    ("ei_export/ei_combined.csv",     "★ CSV Wizard용 (방법 A, 권장)"),
    ("ei_export/ei_combined_raw.csv", "  CSV Wizard용 원시값 버전"),
    ("ei_export/ei_features.csv",     "  CLI 업로더용 특징값 (방법 C)"),
    ("ei_export/ei_labels.csv",       "  CLI 업로더용 라벨 (방법 C)"),
    ("ei_export/individual_samples/", "  개별 파일 방식 (방법 B)"),
    ("ei_export/IMPORT_GUIDE.md",     "  Edge Impulse 업로드 가이드"),
    ("ei_export.zip",                 "  업로드용 ZIP (방법 A/C)"),
    ("ei_export_individual.zip",      "  업로드용 ZIP (방법 B)"),
]

for fpath, desc in output_files:
    try:
        if os.path.isdir(f'./{fpath}'):
            cnt = len(os.listdir(f'./{fpath}'))
            print(f"  📁 {fpath:<42} {desc}  [{cnt}개 파일]")
        else:
            size = os.path.getsize(f'./{fpath}')
            size_str = f"{size//1024} KB" if size > 1024 else f"{size} B"
            print(f"  📄 [{size_str:>7}] {fpath:<42} {desc}")
    except:
        print(f"  ❌ {fpath} (생성 실패)")

print("\n🚀 다음 단계:")
print("   1. ei_export.zip 다운로드")
print("   2. Edge Impulse Studio (studio.edgeimpulse.com) 접속")
print("   3. Data acquisition > Upload data > ei_combined.csv 업로드")
print("   4. CSV Wizard에서 'label' 열을 Label column으로 지정")
print("   5. Impulse Design > Add learning block > Classification")
print("   6. Train → Deploy → Arduino library 다운로드")
