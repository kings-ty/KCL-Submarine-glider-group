# KCL Submarine Glider Group

수중 글라이더/소형 AUV 기반 **수질 모니터링 + TinyML 이상탐지 + LoRa 중계 통신** 프로젝트입니다.  
이 README는 저장소 내 코드/보고서(`submarine_dev/training_report.txt`, `submarine_dev/final_report_sections.md`)를 바탕으로 전체 내용을 통합 정리한 문서입니다.

## 1) 프로젝트 목표

- 수중 환경 센서 데이터(pH, DO, EC, 온도, IMU, 깊이)를 수집
- 엣지 디바이스(ESP32)에서 TinyML로 이상 상태를 실시간 분류
- LoRa 릴레이 및 Azure IoT 연동으로 원격 모니터링
- 이상 탐지 시 STM32 제어기로 긴급 상태(`STATE_EMERGENCY`) 전달

## 2) 시스템 구성 요약

- **제어/센서 계층 (STM32)**  
  수질 센서 및 관성/깊이 데이터를 수집하고 제어 루프를 수행
- **지능 계층 (ESP32 TinyML)**  
  INT8 양자화 모델로 실시간 이상탐지 수행
- **통신 계층 (LoRa 듀얼 ESP32 브리지)**  
  수중 노드 → 수면 릴레이 → 육상 수신기로 데이터 전달
- **모니터링 계층 (Python Gateway + Azure IoT)**  
  JSON 텔레메트리 업링크 및 대시보드/알림 제공
- **전원 구조 (Dual Battery)**  
  추진계와 로직/센서 전원을 분리해 노이즈 영향 최소화

## 3) TinyML 학습 파이프라인

`submarine_dev/step1_eda_tinyml.py` ~ `step9_final_report_tinyml.py`로 단계별 구성:

1. 데이터 탐색/전처리
2. 윈도우 기반 특징 추출
3. 정규화
4. 오토인코더/분류기 학습
5. INT8 양자화 및 임베디드 배포 산출물 생성
6. 최종 리포트 생성

주요 산출물:
- `submarine_dev/auv_classifier_int8.h`
- `submarine_dev/auv_norm_params.h`
- `submarine_dev/confusion_matrix.png`
- `submarine_dev/training_report.txt`

## 4) 핵심 결과 (보고서 기준)

### TinyML 성능
- 모델 구조: `30 → 32 → 16 → 4`
- INT8 모델 크기: **5.64 KB** (FP32 8.61 KB 대비 축소)
- 정확도: **96.03%**
- Macro F1: **0.7967**
- ESP32 추론 지연: **약 1 ms**

### 통신/시스템 검증
- LoRa 100m 구간 PDR: **94.2%**
- 종단 지연(탐지→대시보드): **평균 1.85초**
- HIL 검증: 이상 탐지 후 약 **1.2초** 내 긴급 상태 전환 트리거

## 5) 저장소 구조

- `submarine-test/` : STM32CubeIDE 기반 펌웨어 프로젝트
- `submarine_dev/` : TinyML 학습/양자화/시뮬레이션/ESP32 코드
  - `simulation/` : 글라이더/게이트웨이 시뮬레이션
  - `lora_base_station/` : LoRa 베이스 스테이션 스케치
  - `tflite_test/` : TFLite Micro 추론 테스트
- 루트 Python 스크립트: 통신/시리얼/시뮬레이션 보조 테스트 코드

## 6) 참고 이미지

![test-ucl](https://github.com/user-attachments/assets/fa977f7b-3119-4786-85b9-683699786343)
![lipstick-battery](https://github.com/user-attachments/assets/ebf3c85a-7f7b-41c1-b9d1-114b3c05451d)
![monitoring](https://github.com/user-attachments/assets/d98f4bd2-8ca5-4738-a45c-ae644dd94472)
![WhatsApp Image 2026-04-03 at 17 12 19](https://github.com/user-attachments/assets/746e3bb6-1338-476e-b979-9734b06d545d)
