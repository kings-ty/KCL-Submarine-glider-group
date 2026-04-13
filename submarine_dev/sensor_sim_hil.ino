// /*
//  * ============================================================================
//  *  ESP32 Heltec V3 — TinyML-Ready Firmware v2.0
//  * ============================================================================
//  *
//  *  ✅ 현재 STM32 프로토콜 ("D:x.xx,T:x.xx") 그대로 호환
//  *  ✅ 나중에 STM32가 "TEL:D:...,P:...,R:...,Y:...,M:..." 보내면 파서만 확장
//  *  ✅ 순환 버퍼 → 특징 추출 → ML 분류 → 상태 머신 → LoRa 인사이트 패킷
//  *
//  *  원본 코드에서 바뀐 부분 요약:
//  *  ──────────────────────────────────────────────────────────────
//  *  [변경 1] 샘플링 배열(phSamples[20]) → 순환 버퍼(CircularBuffer) 구조체
//  *  [변경 2] getTrimmedMean() → extractFeatures() (통계 + 상관관계 특징)
//  *  [변경 3] 1분마다 평균 전송 → 6.4초 윈도우마다 ML 분류 실행
//  *  [변경 4] String payload → 32바이트 바이너리 인사이트 패킷
//  *  [변경 5] 상태 머신 추가 (NORMAL → WATCHING → CONFIRMED → EMERGENCY)
//  *  [변경 6] STM32 UART 파서: 현재 "D:,T:" 호환 + 미래 "TEL:" 확장 가능
//  *  [변경 7] 비상 상승 UART 커맨드 추가
//  *  ──────────────────────────────────────────────────────────────
//  */

// #include "LoRaWan_APP.h"
// #include "Arduino.h"
// #include <Wire.h>
// #include "hil_types.h"  // 모든 struct/enum 타입 정의

// // ============================================================
// // 📡 LoRa Settings (원본 그대로)
// // ============================================================
// #define RF_FREQUENCY          868000000
// #define TX_OUTPUT_POWER       14
// #define LORA_BANDWIDTH        0
// #define LORA_SPREADING_FACTOR 7
// #define LORA_CODINGRATE       1
// #define LORA_PREAMBLE_LENGTH  8

// static RadioEvents_t RadioEvents;
// HardwareSerial SerialSTM(2);

// // ============================================================
// // 🎛️ Sensor Pins (원본 그대로)
// // ============================================================
// const int phPin       = 1;
// const int ecPin       = 4;
// const int analogDoPin = 5;
// const int DO_I2C_ADDR = 0x61;

// // ============================================================
// // 전역 인스턴스 (타입 정의는 hil_types.h 참조)
// // ============================================================
// CircularBuffer sensorBuffer;
// FeatureVector  currentFeatures;

// // ============================================================
// // 상태 변수
// // ============================================================
// SystemState sysState;

// // ============================================================
// // STM32에서 받아올 데이터 (원본 확장)
// // ============================================================
// float latestDepth = 0.0;
// float latestTemp  = 0.0;
// bool  stmConnected = false;

// // [미래 확장] STM32가 보내줄 IMU 데이터 — 지금은 0.0
// float latestPitch    = 0.0;
// float latestRoll     = 0.0;
// float latestYaw      = 0.0;
// float latestIMUEnergy = 0.0;  // sqrt(gx²+gy²+gz²)
// uint8_t latestMotorState = 0; // 0=IDLE, 1=MOVING_LEFT, 2=MOVING_RIGHT

// // LoRaInsightPacket, EmergencyCommand → hil_types.h 참조

// // ============================================================
// // 타이밍 변수
// // ============================================================
// uint32_t lastSampleTime   = 0;
// uint32_t lastLoRaSendTime = 0;
// uint32_t missionStartTime = 0;

// const uint32_t LORA_SEND_INTERVAL_MS = 60000; // 1분마다 LoRa 전송 (표면에서)

// // ============================================================
// // 📡 LoRa 콜백 (원본 그대로)
// // ============================================================
// void OnTxDone(void)    { Serial.println("[LoRa] Tx Success!\n"); }
// void OnTxTimeout(void) { Serial.println("[LoRa] Tx Timeout!\n"); }

// // 🔑 Forward declarations (컴파일 에러 방지)
// void sendEmergencyCommand(uint8_t reason);
// void sendLoRaInsight();
// void sendSDLOGtoSTM32();


// // ============================================================
// // 🔧 SETUP
// // ============================================================
// void setup() {
//   Serial.begin(115200);
//   SerialSTM.begin(115200, SERIAL_8N1, 16, 17); // RX=GPIO16 ← STM32 PD8(TX), TX=GPIO17 → STM32 PD9(RX)
//   Mcu.begin(0, 0);

//   pinMode(45, OUTPUT);
//   digitalWrite(45, LOW);
//   delay(100);

//   Wire.begin(41, 42);
//   analogReadResolution(12);

//   // LoRa 초기화 (원본 그대로)
//   RadioEvents.TxDone    = OnTxDone;
//   RadioEvents.TxTimeout = OnTxTimeout;
//   Radio.Init(&RadioEvents);
//   Radio.SetChannel(RF_FREQUENCY);
//   Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
//                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
//                     LORA_PREAMBLE_LENGTH, false,
//                     true, 0, 0, false, 3000);

//   // [NEW] 순환 버퍼 초기화
//   memset(&sensorBuffer, 0, sizeof(sensorBuffer));

//   // [NEW] 상태 머신 초기화
//   memset(&sysState, 0, sizeof(sysState));
//   sysState.alertState = STATE_NORMAL;

//   missionStartTime = millis();

//   Serial.println("\n============================================");
//   Serial.println("  ESP32 TinyML-Ready Firmware v2.0");
//   Serial.println("  Window: 64 samples @ 10Hz = 6.4s");
//   Serial.println("  Stride: 16 samples = 1.6s per inference");
//   Serial.println("============================================\n");
// }


// // ============================================================
// // 📐 수학 헬퍼 함수들
// // ============================================================

// // 배열 평균
// float calcMean(float* arr, int size) {
//   float sum = 0;
//   for (int i = 0; i < size; i++) sum += arr[i];
//   return sum / size;
// }

// // 배열 표준편차
// float calcStd(float* arr, int size, float mean) {
//   float sumSq = 0;
//   for (int i = 0; i < size; i++) {
//     float diff = arr[i] - mean;
//     sumSq += diff * diff;
//   }
//   return sqrtf(sumSq / size);
// }

// // 배열 RMS
// float calcRMS(float* arr, int size) {
//   float sumSq = 0;
//   for (int i = 0; i < size; i++) sumSq += arr[i] * arr[i];
//   return sqrtf(sumSq / size);
// }

// // 선형 회귀 기울기 (drift 방향 감지)
// float calcSlope(float* arr, int size) {
//   float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
//   for (int i = 0; i < size; i++) {
//     sumX  += i;
//     sumY  += arr[i];
//     sumXY += i * arr[i];
//     sumX2 += i * i;
//   }
//   float denom = (size * sumX2 - sumX * sumX);
//   if (fabsf(denom) < 1e-10) return 0.0f;
//   return (size * sumXY - sumX * sumY) / denom;
// }

// // 배열 최대값
// float calcMax(float* arr, int size) {
//   float maxVal = arr[0];
//   for (int i = 1; i < size; i++) {
//     if (arr[i] > maxVal) maxVal = arr[i];
//   }
//   return maxVal;
// }

// // 피어슨 상관계수
// float calcCorrelation(float* x, float* y, int size) {
//   float mx = calcMean(x, size);
//   float my = calcMean(y, size);
//   float num = 0, dx2 = 0, dy2 = 0;
//   for (int i = 0; i < size; i++) {
//     float dx = x[i] - mx;
//     float dy = y[i] - my;
//     num += dx * dy;
//     dx2 += dx * dx;
//     dy2 += dy * dy;
//   }
//   float denom = sqrtf(dx2 * dy2);
//   if (denom < 1e-10) return 0.0f;
//   return num / denom;
// }


// // ============================================================
// // [변경 2] 특징 추출 — 순환 버퍼에서 윈도우를 꺼내서 30개 특징 계산
// // ============================================================
// // 원본: getTrimmedMean(phSamples, 20, 2)  → 단순 평균 1개
// // 변경: 윈도우 내 7채널 × 통계 + 상관관계 = 30개 특징

// void extractFeatures(FeatureVector* fv) {
//   // 임시 배열: 윈도우 데이터를 채널별로 분리
//   float ch_data[NUM_CHANNELS][WINDOW_SIZE];

//   // 순환 버퍼에서 최근 WINDOW_SIZE개 샘플 복사
//   for (int i = 0; i < WINDOW_SIZE; i++) {
//     int idx = (sensorBuffer.writeIndex - WINDOW_SIZE + i + WINDOW_SIZE) % WINDOW_SIZE;
//     for (int c = 0; c < NUM_CHANNELS; c++) {
//       ch_data[c][i] = sensorBuffer.data[idx][c];
//     }
//   }

//   // ── 시간 도메인 특징: 각 WQ 채널 ──
//   fv->ph_mean  = calcMean(ch_data[CH_PH], WINDOW_SIZE);
//   fv->ph_std   = calcStd(ch_data[CH_PH], WINDOW_SIZE, fv->ph_mean);
//   fv->ph_slope = calcSlope(ch_data[CH_PH], WINDOW_SIZE);
//   fv->ph_rms   = calcRMS(ch_data[CH_PH], WINDOW_SIZE);

//   fv->ec_mean  = calcMean(ch_data[CH_EC], WINDOW_SIZE);
//   fv->ec_std   = calcStd(ch_data[CH_EC], WINDOW_SIZE, fv->ec_mean);
//   fv->ec_slope = calcSlope(ch_data[CH_EC], WINDOW_SIZE);
//   fv->ec_rms   = calcRMS(ch_data[CH_EC], WINDOW_SIZE);

//   fv->do_mean  = calcMean(ch_data[CH_DO], WINDOW_SIZE);
//   fv->do_std   = calcStd(ch_data[CH_DO], WINDOW_SIZE, fv->do_mean);
//   fv->do_slope = calcSlope(ch_data[CH_DO], WINDOW_SIZE);
//   fv->do_rms   = calcRMS(ch_data[CH_DO], WINDOW_SIZE);

//   fv->o2_mean  = calcMean(ch_data[CH_O2], WINDOW_SIZE);
//   fv->o2_std   = calcStd(ch_data[CH_O2], WINDOW_SIZE, fv->o2_mean);
//   fv->o2_slope = calcSlope(ch_data[CH_O2], WINDOW_SIZE);
//   fv->o2_rms   = calcRMS(ch_data[CH_O2], WINDOW_SIZE);

//   // ── Depth & Temp ──
//   fv->depth_mean  = calcMean(ch_data[CH_DEPTH], WINDOW_SIZE);
//   fv->depth_std   = calcStd(ch_data[CH_DEPTH], WINDOW_SIZE, fv->depth_mean);
//   fv->depth_slope = calcSlope(ch_data[CH_DEPTH], WINDOW_SIZE);

//   fv->temp_mean  = calcMean(ch_data[CH_TEMP], WINDOW_SIZE);
//   fv->temp_std   = calcStd(ch_data[CH_TEMP], WINDOW_SIZE, fv->temp_mean);
//   fv->temp_slope = calcSlope(ch_data[CH_TEMP], WINDOW_SIZE);

//   // ── IMU Energy ──
//   fv->imu_energy_mean = calcMean(ch_data[CH_IMU_E], WINDOW_SIZE);
//   fv->imu_energy_std  = calcStd(ch_data[CH_IMU_E], WINDOW_SIZE, fv->imu_energy_mean);
//   fv->imu_energy_max  = calcMax(ch_data[CH_IMU_E], WINDOW_SIZE);

//   // ── Cross-Channel 특징 (핵심!) ──
//   // pH-Temperature 상관관계: 깨끗한 프로브 = ~-0.02/°C 관계
//   fv->ph_temp_corr = calcCorrelation(ch_data[CH_PH], ch_data[CH_TEMP], WINDOW_SIZE);

//   // pH 잔차: actual_pH - expected_pH(temp)
//   // 간단한 선형 모델: expected_pH ≈ 7.0 + (-0.02 × (temp - 20))
//   // (나중에 실제 교정 데이터로 계수 조정)
//   float ph_residuals[WINDOW_SIZE];
//   for (int i = 0; i < WINDOW_SIZE; i++) {
//     float expected_ph = 7.0f + (-0.02f * (ch_data[CH_TEMP][i] - 20.0f));
//     ph_residuals[i] = ch_data[CH_PH][i] - expected_ph;
//   }
//   fv->ph_residual_mean = calcMean(ph_residuals, WINDOW_SIZE);
//   fv->ph_residual_std  = calcStd(ph_residuals, WINDOW_SIZE, fv->ph_residual_mean);

//   // DO 잔차: actual_DO - expected_DO(depth, temp)
//   // Henry's Law 근사: DO_saturated ≈ 14.6 - 0.39*temp + 0.007*depth
//   // (나중에 실제 교정 데이터로 계수 조정)
//   float do_residuals[WINDOW_SIZE];
//   for (int i = 0; i < WINDOW_SIZE; i++) {
//     float expected_do = 14.6f - 0.39f * ch_data[CH_TEMP][i] + 0.007f * ch_data[CH_DEPTH][i];
//     do_residuals[i] = ch_data[CH_DO][i] - expected_do;
//   }
//   fv->do_residual_mean = calcMean(do_residuals, WINDOW_SIZE);
//   fv->do_residual_std  = calcStd(do_residuals, WINDOW_SIZE, fv->do_residual_mean);
// }


// // ============================================================
// // [변경 3] ML 분류기 — 지금은 규칙 기반 플레이스홀더
// // ============================================================
// // 🔑 나중에 이 함수 하나만 emlearn의 auv_model_predict()로 교체!
// //
// // 학습 데이터 수집 후 교체 순서:
// //   1. Python에서 sklearn RandomForest 학습
// //   2. emlearn으로 변환 → auv_classifier.h 생성
// //   3. 이 함수를 #include "auv_classifier.h" + auv_model_predict()로 교체
// //   4. 끝! 나머지 파이프라인은 그대로 동작

// MLClass runMLClassifier(FeatureVector* fv) {

//   // ── 규칙 1: IMU 에너지 기반 모션 아티팩트 감지 ──
//   // (지금은 STM32에서 IMU 안 보내주니까 항상 0 → 이 규칙은 비활성)
//   if (fv->imu_energy_mean > 2.0f) {
//     return CLASS_MOTION_ARTIFACT;
//   }

//   // ── 규칙 2: 센서 고장 감지 (biofouling) ──
//   // pH가 급격히 드리프트하는데 (slope 큼), 온도는 안정적 → 센서 문제
//   if (fabsf(fv->ph_slope) > 0.01f && fv->temp_std < 0.5f) {
//     return CLASS_SENSOR_FAULT;
//   }
//   // pH 잔차가 비정상적으로 큼 → 기대값에서 크게 벗어남
//   if (fabsf(fv->ph_residual_mean) > 1.0f) {
//     return CLASS_SENSOR_FAULT;
//   }
//   // EC가 갑자기 0 근처 또는 std 극단적 → 전극 문제
//   if (fv->ec_mean < 0.01f || fv->ec_std > 100.0f) {
//     return CLASS_SENSOR_FAULT;
//   }

//   // ── 규칙 3: 환경 이상 감지 ──
//   // 모든 WQ 센서가 동시에 변동 (온도/수심도 변하면서) → 진짜 환경 변화
//   bool multiSensorShift = (fv->ph_std > 0.3f) && (fv->do_std > 0.5f) && (fv->ec_std > 20.0f);
//   bool envContext = (fv->depth_std > 0.5f) || (fv->temp_std > 1.0f);
//   if (multiSensorShift && envContext) {
//     return CLASS_ENV_ANOMALY;
//   }

//   return CLASS_NORMAL;
// }


// // ============================================================
// // [변경 5] 상태 머신 업데이트
// // ============================================================
// void updateStateMachine(MLClass mlResult) {
//   uint32_t now = millis();

//   if (mlResult == CLASS_NORMAL || mlResult == CLASS_MOTION_ARTIFACT) {
//     // 정상 또는 모션 아티팩트 → 연속 이상 카운터 감소
//     if (sysState.consecutiveAnomalies > 0) {
//       sysState.consecutiveAnomalies--;
//     }
//     // WATCHING 상태에서 30초간 이상 없으면 NORMAL로 복귀
//     if (sysState.alertState == STATE_WATCHING &&
//         (now - sysState.watchStartTime > 30000) &&
//         sysState.consecutiveAnomalies == 0) {
//       sysState.alertState = STATE_NORMAL;
//       Serial.println("[STATE] → NORMAL (recovered)");
//     }
//   }
//   else {
//     // SENSOR_FAULT 또는 ENV_ANOMALY
//     sysState.consecutiveAnomalies++;
//     sysState.anomalyCountThisDive++;
//     sysState.anomalyHistogram[mlResult]++;

//     // 센서 고장 플래그 설정
//     if (mlResult == CLASS_SENSOR_FAULT) {
//       // 어떤 센서가 문제인지 특징으로 판단
//       if (fabsf(currentFeatures.ph_slope) > 0.01f)   sysState.sensorFaultFlags |= 0x80; // pH
//       if (currentFeatures.ec_mean < 0.01f)            sysState.sensorFaultFlags |= 0x40; // EC
//       if (fabsf(currentFeatures.do_residual_mean) > 2.0f) sysState.sensorFaultFlags |= 0x20; // DO
//     }

//     // ── 상태 전이 ──
//     switch (sysState.alertState) {
//       case STATE_NORMAL:
//         if (sysState.consecutiveAnomalies >= 3) {
//           sysState.alertState = STATE_WATCHING;
//           sysState.watchStartTime = now;
//           Serial.println("[STATE] → WATCHING (3 consecutive anomalies)");
//         }
//         break;

//       case STATE_WATCHING:
//         if (sysState.consecutiveAnomalies >= 8) {
//           sysState.alertState = STATE_CONFIRMED;
//           Serial.println("[STATE] → CONFIRMED (persistent anomaly)");
//         }
//         break;

//       case STATE_CONFIRMED:
//         // 2개 이상 센서 동시 고장 → 비상
//         {
//           uint8_t faultCount = 0;
//           for (int i = 0; i < 8; i++) {
//             if (sysState.sensorFaultFlags & (1 << i)) faultCount++;
//           }
//           if (faultCount >= 2 || sysState.consecutiveAnomalies >= 15) {
//             sysState.alertState = STATE_EMERGENCY;
//             Serial.println("[STATE] ⚠️ → EMERGENCY ASCENT!");
//             sendEmergencyCommand(0x01); // reason: sensor failure
//           }
//         }
//         break;

//       case STATE_EMERGENCY:
//         // 이미 비상 — 추가 조치 없음 (리셋으로만 복구)
//         break;
//     }
//   }
// }


// // ============================================================
// // [변경 7] 비상 상승 커맨드 전송
// // ============================================================
// void sendEmergencyCommand(uint8_t reason) {
//   EmergencyCommand cmd;
//   cmd.header   = 0xAA55;
//   cmd.command  = 0xEA;
//   cmd.reason   = reason;
//   cmd.checksum = cmd.command ^ cmd.reason;

//   SerialSTM.write((uint8_t*)&cmd, sizeof(cmd));

//   Serial.printf("[EMERGENCY] Sent to STM32: reason=0x%02X\n", reason);
// }


// // ============================================================
// // [변경 4] LoRa 인사이트 패킷 전송
// // ============================================================
// uint16_t calcCRC16(uint8_t* data, uint16_t len) {
//   uint16_t crc = 0xFFFF;
//   for (uint16_t i = 0; i < len; i++) {
//     crc ^= data[i];
//     for (int j = 0; j < 8; j++) {
//       if (crc & 1) crc = (crc >> 1) ^ 0xA001;
//       else crc >>= 1;
//     }
//   }
//   return crc;
// }

// void sendLoRaInsight() {
//   LoRaInsightPacket pkt;
//   memset(&pkt, 0, sizeof(pkt));

//   pkt.timestamp     = (millis() - missionStartTime) / 1000;
//   pkt.diveCount     = sysState.diveCount;
//   pkt.maxDepth_dm   = (uint8_t)(sysState.maxDepthThisDive * 10.0f);  // m → dm
//   pkt.tempSurface_c = (int8_t)latestTemp;
//   pkt.tempBottom_c  = (int8_t)latestTemp;  // 나중에 depth별 분리
//   pkt.pH_mean_x10   = (uint8_t)constrain((int)(currentFeatures.ph_mean * 10.0f), 0, 255);
//   pkt.DO_mean_x10   = (uint8_t)constrain((int)(currentFeatures.do_mean * 10.0f), 0, 255);
//   pkt.EC_mean       = (uint16_t)currentFeatures.ec_mean;
//   pkt.anomalyCount  = sysState.anomalyCountThisDive;
//   pkt.faultFlags    = sysState.sensorFaultFlags;
//   pkt.confidenceFlags = 0xFF;  // 나중에 센서별 건강도 계산
//   pkt.alertLevel    = (uint8_t)sysState.alertState;
//   memcpy(pkt.anomalyClasses, sysState.anomalyHistogram, 4);

//   // CRC 계산 (CRC 필드 제외)
//   pkt.crc16 = calcCRC16((uint8_t*)&pkt, sizeof(pkt) - 2);

//   // LoRa 전송!
//   Radio.Send((uint8_t*)&pkt, sizeof(pkt));

//   Serial.printf("[LoRa] Insight packet sent: %d bytes, alert=%d, anomalies=%d\n",
//                 sizeof(pkt), pkt.alertLevel, pkt.anomalyCount);
// }

// // 원본 호환: String 형식 SDLOG도 병행 전송
// void sendSDLOGtoSTM32() {
//   // STM32가 현재 이해하는 포맷 그대로 유지
//   String dataMsg = "SDLOG:D:" + String(latestDepth, 2) +
//                    ",T:" + String(latestTemp, 2) +
//                    ",PH:" + String(currentFeatures.ph_mean, 3) +
//                    ",EC:" + String(currentFeatures.ec_mean, 3) +
//                    ",aDO:" + String(currentFeatures.do_mean, 3) +
//                    ",O2:" + String(currentFeatures.o2_mean, 3) +
//                    ",ML:" + String((int)sysState.alertState) +
//                    ",ANM:" + String(sysState.anomalyCountThisDive);
//   SerialSTM.println(dataMsg);
//   Serial.println("[SDLOG] " + dataMsg);
// }


// // ============================================================
// // [변경 6] STM32 UART 파서 — 현재 호환 + 미래 확장
// // ============================================================
// void parseSTM32Data(String msg) {
//   msg.trim();

//   // ── 현재 포맷: "D:1.50,T:20.30" ──
//   if (msg.startsWith("D:")) {
//     int commaIndex = msg.indexOf(',');
//     if (commaIndex > 0) {
//       latestDepth = msg.substring(2, commaIndex).toFloat();
//       // "T:" 찾기
//       int tIndex = msg.indexOf("T:", commaIndex);
//       if (tIndex > 0) {
//         latestTemp = msg.substring(tIndex + 2).toFloat();
//       }
//       stmConnected = true;

//       // 최대 수심 추적
//       if (latestDepth > sysState.maxDepthThisDive) {
//         sysState.maxDepthThisDive = latestDepth;
//       }
//     }
//   }

//   // ── 미래 포맷: "TEL:D:1.50,T:20.30,P:2.10,R:-0.80,Y:145.3,M:0" ──
//   // STM32 팀이 준비되면 이 블록을 활성화
//   else if (msg.startsWith("TEL:")) {
//     String payload = msg.substring(4);

//     // D: (Depth)
//     int idx = payload.indexOf("D:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestDepth = payload.substring(idx + 2, end > 0 ? end : payload.length()).toFloat();
//     }
//     // T: (Temp)
//     idx = payload.indexOf("T:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestTemp = payload.substring(idx + 2, end > 0 ? end : payload.length()).toFloat();
//     }
//     // P: (Pitch)
//     idx = payload.indexOf("P:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestPitch = payload.substring(idx + 2, end > 0 ? end : payload.length()).toFloat();
//     }
//     // R: (Roll)
//     idx = payload.indexOf("R:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestRoll = payload.substring(idx + 2, end > 0 ? end : payload.length()).toFloat();
//     }
//     // Y: (Yaw)
//     idx = payload.indexOf("Y:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestYaw = payload.substring(idx + 2, end > 0 ? end : payload.length()).toFloat();
//     }
//     // M: (Motor state)
//     idx = payload.indexOf("M:");
//     if (idx >= 0) {
//       int end = payload.indexOf(',', idx);
//       latestMotorState = (uint8_t)payload.substring(idx + 2, end > 0 ? end : payload.length()).toInt();
//     }

//     stmConnected = true;

//     // IMU energy 계산 (Gyro 없으면 Pitch/Roll 변화율로 근사)
//     static float prevPitch = 0, prevRoll = 0;
//     float dp = latestPitch - prevPitch;
//     float dr = latestRoll - prevRoll;
//     latestIMUEnergy = sqrtf(dp * dp + dr * dr);
//     prevPitch = latestPitch;
//     prevRoll  = latestRoll;

//     if (latestDepth > sysState.maxDepthThisDive) {
//       sysState.maxDepthThisDive = latestDepth;
//     }
//   }
// }


// // ============================================================
// // 디버그: 특징 벡터 시리얼 출력
// // ============================================================
// void printFeaturesSummary() {
//   Serial.println("──── Feature Vector ────");
//   Serial.printf("  pH:    mean=%.3f std=%.4f slope=%.5f\n",
//                 currentFeatures.ph_mean, currentFeatures.ph_std, currentFeatures.ph_slope);
//   Serial.printf("  EC:    mean=%.3f std=%.4f slope=%.5f\n",
//                 currentFeatures.ec_mean, currentFeatures.ec_std, currentFeatures.ec_slope);
//   Serial.printf("  DO:    mean=%.3f std=%.4f slope=%.5f\n",
//                 currentFeatures.do_mean, currentFeatures.do_std, currentFeatures.do_slope);
//   Serial.printf("  Depth: mean=%.2f  Temp: mean=%.2f\n",
//                 currentFeatures.depth_mean, currentFeatures.temp_mean);
//   Serial.printf("  pH-Temp corr: %.3f\n", currentFeatures.ph_temp_corr);
//   Serial.printf("  pH residual:  mean=%.3f std=%.3f\n",
//                 currentFeatures.ph_residual_mean, currentFeatures.ph_residual_std);
//   Serial.printf("  DO residual:  mean=%.3f std=%.3f\n",
//                 currentFeatures.do_residual_mean, currentFeatures.do_residual_std);
//   Serial.println("────────────────────────");
// }


// // ============================================================
// // 🔄 MAIN LOOP
// // ============================================================
// void loop() {

//   // ─── 1. STM32 UART 수신 (항상, 원본과 동일한 구조) ───
//   while (SerialSTM.available()) {
//     String incomingMsg = SerialSTM.readStringUntil('\n');
//     parseSTM32Data(incomingMsg);  // [변경 6] 확장 가능 파서 사용
//   }

//   // ─── 2. 10Hz 센서 샘플링 (원본: 3초 → 100ms로 변경) ───
//   if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
//     lastSampleTime = millis();

//     // 센서 읽기 (원본과 동일한 핀/방법)
//     float phVoltage = analogRead(phPin)       * (3.3f / 4095.0f);
//     float ecVoltage = analogRead(ecPin)       * (3.3f / 4095.0f);
//     float doVoltage = analogRead(analogDoPin) * (3.3f / 4095.0f);
//     float i2cO2     = 0.0f;  // I2C 산소센서 (원본처럼 주석 상태 유지)

//     // 순환 버퍼에 저장
//     uint16_t wi = sensorBuffer.writeIndex;
//     sensorBuffer.data[wi][CH_PH]    = phVoltage;
//     sensorBuffer.data[wi][CH_EC]    = ecVoltage;
//     sensorBuffer.data[wi][CH_DO]    = doVoltage;
//     sensorBuffer.data[wi][CH_O2]    = i2cO2;
//     sensorBuffer.data[wi][CH_DEPTH] = latestDepth;    // STM32에서 받은 최신값
//     sensorBuffer.data[wi][CH_TEMP]  = latestTemp;
//     sensorBuffer.data[wi][CH_IMU_E] = latestIMUEnergy; // 미래: STM32 IMU

//     sensorBuffer.writeIndex = (wi + 1) % WINDOW_SIZE;
//     sensorBuffer.totalSamples++;

//     // ─── 3. 윈도우가 차면 → 특징 추출 → ML 분류 ───
//     // 첫 64개 채우기 전에는 분류 안 함
//     // stride=16: 매 16 샘플마다 분류 실행 (1.6초마다)
//     if (sensorBuffer.totalSamples >= WINDOW_SIZE &&
//         sensorBuffer.totalSamples % WINDOW_STRIDE == 0) {

//       // [변경 2] 특징 추출
//       extractFeatures(&currentFeatures);

//       // [변경 3] ML 분류 (지금은 규칙 기반, 나중에 emlearn 교체)
//       MLClass result = runMLClassifier(&currentFeatures);

//       // [변경 5] 상태 머신 업데이트
//       updateStateMachine(result);

//       // 디버그 출력
//       const char* classNames[] = {"NORMAL", "MOTION_ART", "SENSOR_FAULT", "ENV_ANOMALY"};
//       Serial.printf("[ML] Class: %s | State: %d | Consecutive: %d\n",
//                     classNames[result],
//                     sysState.alertState,
//                     sysState.consecutiveAnomalies);

//       // 자세한 특징 출력 (10번에 한 번)
//       if ((sensorBuffer.totalSamples / WINDOW_STRIDE) % 10 == 0) {
//         printFeaturesSummary();
//       }
//     }
//   }

//   // ─── 4. 주기적 데이터 전송 (LoRa + SDLOG) ───
//   if (millis() - lastLoRaSendTime >= LORA_SEND_INTERVAL_MS) {
//     lastLoRaSendTime = millis();

//     // 최소 한 번 이상 특징 추출이 완료되었으면 전송
//     if (sensorBuffer.totalSamples >= WINDOW_SIZE) {
//       // [변경 4] 바이너리 인사이트 패킷 LoRa 전송
//       sendLoRaInsight();

//       // 원본 호환: STM32에 SDLOG 전송 (현재 STM32가 이해하는 포맷)
//       sendSDLOGtoSTM32();
//     }
//   }

//   // ─── 5. LoRa 백그라운드 프로세스 (원본 그대로) ───
//   Radio.IrqProcess();
// }


// /*
//  * ============================================================================
//  * 📋 다음 단계 체크리스트
//  * ============================================================================
//  *
//  * ✅ Phase 1 (지금 완료):
//  *    - 순환 버퍼 + 특징 추출 + 규칙 기반 분류 + 상태 머신
//  *    - 현재 STM32 프로토콜 100% 호환
//  *
//  * 📅 Phase 2 (STM32 통합 후 — 다음주 목요일 이후):
//  *    - STM32에서 "TEL:" 포맷으로 IMU + Motor 데이터 전송
//  *    - latestPitch/Roll/Yaw/MotorState가 실제 값으로 채워짐
//  *    - IMU energy 특징이 활성화됨 → 모션 아티팩트 감지 가능
//  *
//  * 📅 Phase 3 (데이터 수집 2-4시간 후):
//  *    - SD 카드에서 CSV 데이터 추출
//  *    - Python으로 라벨링 + sklearn RandomForest 학습
//  *    - emlearn 변환 → auv_classifier.h 생성
//  *    - runMLClassifier() 함수 교체:
//  *
//  *      #include "auv_classifier.h"
//  *      MLClass runMLClassifier(FeatureVector* fv) {
//  *        float features[30];
//  *        memcpy(features, fv, sizeof(float) * 30);
//  *        return (MLClass)auv_model_predict(features, 30);
//  *      }
//  *
//  * 📅 Phase 4 (비상 상승 통합):
//  *    - STM32에 EmergencyCommand 수신 핸들러 추가
//  *    - 0xAA55 + 0xEA 수신 시 → 모터 중립 + 부력 상승
//  *
//  * ============================================================================
//  */