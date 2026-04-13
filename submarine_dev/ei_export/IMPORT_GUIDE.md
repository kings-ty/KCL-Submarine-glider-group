# Edge Impulse Export - AUV Anomaly Detection
Dataset: InWaterSense (Ahmedi 2021) | Classes: 4 | Features: 30 | Windows: 1689

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
edge-impulse-uploader --format csv \
  --label-path ./ei_labels.csv \
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
  NORMAL              :   349개 (20.7%)
  MOTION_ARTIFACT     :     0개 (0.0%)
  SENSOR_FAULT        :  1329개 (78.7%)
  ENV_ANOMALY         :    11개 (0.7%)
