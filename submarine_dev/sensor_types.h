#pragma once
#include <Arduino.h>

#define NUM_CHANNELS 7
#define WINDOW_SIZE 64
#define WINDOW_STRIDE 16
#define SAMPLE_INTERVAL_MS 3000
#define NUM_FEATURES 30

enum ChannelIndex {
  CH_PH   = 0,
  CH_EC   = 1,
  CH_DO   = 2,
  CH_O2   = 3,
  CH_DEPTH   = 4,
  CH_TEMP   = 5,
  CH_IMU_E   = 6 // IMU energy sqrt(gx^2+gy^2+gz^2)
};

struct CircularBuffer {
  float data[WINDOW_SIZE][NUM_CHANNELS];
  uint16_t writeIndex;
  uint32_t totalSamples;
};

struct FeatureVector {
  // 각 WQ 센서: mean, std, slope, rms (4 × 4 = 16)
  float ph_mean, ph_std, ph_slope, ph_rms;
  float ec_mean, ec_std, ec_slope, ec_rms;
  float do_mean, do_std, do_slope, do_rms;
  float o2_mean, o2_std, o2_slope, o2_rms;

  // Depth & Temp: mean, std, slope (3 × 2 = 6)
  float depth_mean, depth_std, depth_slope;
  float temp_mean,  temp_std,  temp_slope;

  // IMU energy: mean, std, max (3)
  float imu_energy_mean, imu_energy_std, imu_energy_max;

  // Cross-channel (5) — 센서 간 물리적 관계
  float ph_temp_corr;         // pH-온도 상관계수
  float ph_residual_mean;     // pH - expected_pH(temp) 잔차
  float ph_residual_std;
  float do_residual_mean;     // DO - expected_DO(depth,temp) 잔차
  float do_residual_std;
};

enum AlertState {
  STATE_NORMAL    = 0,   // 정상 작동
  STATE_WATCHING  = 1,   // 이상 감지됨, 감시 중
  STATE_CONFIRMED = 2,   // 이상 확정, LoRa 알림 큐
  STATE_EMERGENCY = 3    // 비상! STM32에 상승 명령
};

// ML 분류 결과
enum MLClass {
  CLASS_NORMAL         = 0,
  CLASS_MOTION_ARTIFACT = 1,
  CLASS_SENSOR_FAULT   = 2,
  CLASS_ENV_ANOMALY    = 3
};

struct SystemState {
  AlertState  alertState;
  uint8_t     consecutiveAnomalies;   // 연속 이상 횟수
  uint32_t    watchStartTime;         // WATCHING 진입 시각
  uint8_t     anomalyCountThisDive;   // 이번 다이브 총 이상 횟수
  uint8_t     anomalyHistogram[4];    // 클래스별 히스토그램
  uint8_t     sensorFaultFlags;       // 비트필드: [pH|EC|DO|O2|_|_|_|_]
  float       maxDepthThisDive;       // 이번 다이브 최대 수심
  uint16_t    diveCount;              // 총 다이브 횟수
};

// STM32 비상 프로토콜 커맨드
struct __attribute__((packed)) EmergencyCommand {
  uint16_t header;    // 0xAA55
  uint8_t  command;   // 0xEA = Emergency Ascent
  uint8_t  reason;    // 0x01=sensor_fail, 0x02=extreme_anomaly
  uint8_t  checksum;  // XOR(command, reason)
};

// LoRa 인사이트 패킷 전송용
struct __attribute__((packed)) LoRaInsightPacket {
  uint32_t timestamp;         // 4B — 미션 시간 (초)
  uint8_t  diveCount;         // 1B
  uint8_t  maxDepth_dm;       // 1B — 최대 수심 (데시미터, 0-255 = 0~25.5m)
  int8_t   tempSurface_c;     // 1B — 표면 온도
  int8_t   tempBottom_c;      // 1B — 바닥 온도
  uint8_t  pH_mean_x10;       // 1B — pH × 10 (70 = 7.0)
  uint8_t  DO_mean_x10;       // 1B — DO × 10
  uint16_t EC_mean;           // 2B — EC (µS/cm)
  uint8_t  anomalyCount;      // 1B
  uint8_t  faultFlags;        // 1B — 비트필드
  uint8_t  confidenceFlags;   // 1B — 센서별 건강도
  uint8_t  alertLevel;        // 1B — 0=OK, 1=WATCH, 2=ALERT, 3=EMRG
  uint8_t  anomalyClasses[4]; // 4B — 클래스별 히스토그램
  uint8_t  depthProfile[10];  // 10B — 5 depth bins × 2 bytes
  uint16_t crc16;             // 2B — 무결성 검사
};
