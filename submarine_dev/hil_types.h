#pragma once
#include <Arduino.h>

// ============================================================
// 채널 & 윈도우 설정
// ============================================================
#define NUM_CHANNELS        7
#define WINDOW_SIZE         64
#define WINDOW_STRIDE       16
#define SAMPLE_INTERVAL_MS  100
#define NUM_FEATURES        30

// ============================================================
// 채널 인덱스
// ============================================================
enum ChannelIndex {
  CH_PH    = 0,
  CH_EC    = 1,
  CH_DO    = 2,
  CH_O2    = 3,
  CH_DEPTH = 4,
  CH_TEMP  = 5,
  CH_IMU_E = 6
};

// ============================================================
// 순환 버퍼
// ============================================================
struct CircularBuffer {
  float    data[WINDOW_SIZE][NUM_CHANNELS];
  uint16_t writeIndex;
  uint32_t totalSamples;
};

// ============================================================
// 특징 벡터 (30개 특징)
// ============================================================
struct FeatureVector {
  float ph_mean,  ph_std,  ph_slope,  ph_rms;
  float ec_mean,  ec_std,  ec_slope,  ec_rms;
  float do_mean,  do_std,  do_slope,  do_rms;
  float o2_mean,  o2_std,  o2_slope,  o2_rms;
  float depth_mean, depth_std, depth_slope;
  float temp_mean,  temp_std,  temp_slope;
  float imu_energy_mean, imu_energy_std, imu_energy_max;
  float ph_temp_corr;
  float ph_residual_mean, ph_residual_std;
  float do_residual_mean, do_residual_std;
};

// ============================================================
// 상태 머신
// ============================================================
enum AlertState {
  STATE_NORMAL    = 0,
  STATE_WATCHING  = 1,
  STATE_CONFIRMED = 2,
  STATE_EMERGENCY = 3
};

enum MLClass {
  CLASS_NORMAL          = 0,
  CLASS_MOTION_ARTIFACT = 1,
  CLASS_SENSOR_FAULT    = 2,
  CLASS_ENV_ANOMALY     = 3
};

struct SystemState {
  AlertState alertState;
  uint8_t    consecutiveAnomalies;
  uint32_t   watchStartTime;
  uint8_t    anomalyCountThisDive;
  uint8_t    anomalyHistogram[4];
  uint8_t    sensorFaultFlags;
  float      maxDepthThisDive;
  uint16_t   diveCount;
};

// ============================================================
// LoRa 인사이트 패킷 (32 bytes)
// ============================================================
struct __attribute__((packed)) LoRaInsightPacket {
  uint32_t timestamp;
  uint8_t  diveCount;
  uint8_t  maxDepth_dm;
  int8_t   tempSurface_c;
  int8_t   tempBottom_c;
  uint8_t  pH_mean_x10;
  uint8_t  DO_mean_x10;
  uint16_t EC_mean;
  uint8_t  anomalyCount;
  uint8_t  faultFlags;
  uint8_t  confidenceFlags;
  uint8_t  alertLevel;
  uint8_t  anomalyClasses[4];
  uint8_t  depthProfile[10];
  uint16_t crc16;
};

// ============================================================
// 비상 상승 커맨드 (5 bytes)
// ============================================================
struct __attribute__((packed)) EmergencyCommand {
  uint16_t header;    // 0xAA55
  uint8_t  command;   // 0xEA
  uint8_t  reason;    // 0x01 or 0x02
  uint8_t  checksum;  // command ^ reason
};
