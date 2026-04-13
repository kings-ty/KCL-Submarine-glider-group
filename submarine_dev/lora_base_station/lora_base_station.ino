/*
 * LoRa Base Station — 양방향 커맨드-컨트롤
 * Serial에서 입력 받아 LoRa로 AUV에 명령 전송
 * AUV에서 오는 Insight 패킷 수신 및 출력
 *
 * 사용법:
 *   Serial Monitor에서 입력:
 *     RUNNING   → AUV 주행 시작 명령
 *     EMERGENCY → AUV 비상 상승 명령
 *     STATUS    → 상태 요청
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"

#define RF_FREQUENCY          868000000
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8

// ─── 커맨드 패킷 (4 bytes) ───
#define CMD_MAGIC    0xBC
#define CMD_RUNNING  0x01
#define CMD_EMERGENCY 0x02
#define CMD_STATUS   0x03

// ─── LoRaInsightPacket (수신용) ───
#pragma pack(push, 1)
typedef struct {
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
} LoRaInsightPacket_t;  // 32 bytes

typedef struct {
  uint8_t magic;    // 0xBC
  uint8_t command;  // CMD_xxx
  uint8_t param;    // 예비
  uint8_t crc;      // magic ^ command ^ param
} CmdPacket_t;      // 4 bytes
#pragma pack(pop)

static RadioEvents_t RadioEvents;
static bool txBusy = false;
const char* alertNames[] = {"NORMAL", "WATCHING", "CONFIRMED", "EMERGENCY"};

// ─── 커맨드 패킷 전송 ───
void sendCommand(uint8_t cmd, uint8_t param = 0) {
  CmdPacket_t pkt;
  pkt.magic   = CMD_MAGIC;
  pkt.command = cmd;
  pkt.param   = param;
  pkt.crc     = CMD_MAGIC ^ cmd ^ param;

  txBusy = true;
  Radio.Send((uint8_t*)&pkt, sizeof(CmdPacket_t));
}

// ─── TX 완료 → 다시 수신 모드 ───
void OnTxDone(void) {
  txBusy = false;
  Serial.println("✅ Command sent! Listening...");
  Radio.Rx(0);
}

void OnTxTimeout(void) {
  txBusy = false;
  Serial.println("❌ Tx Timeout");
  Radio.Rx(0);
}

// ─── AUV로부터 Insight 패킷 수신 (인터럽트 컨텍스트, 스택 작음) ───
volatile bool rxPending = false;
volatile uint16_t rxSize = 0;
volatile int16_t rxRssi = 0;
volatile int8_t rxSnr = 0;
uint8_t rxBuf[256];

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size <= sizeof(rxBuf)) {
    memcpy(rxBuf, payload, size);
    rxSize = size;
    rxRssi = rssi;
    rxSnr = snr;
    rxPending = true;
  }
  if (!txBusy) Radio.Rx(0);
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(0, 0);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  // RX 설정
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    0, false, 0, true, false, 0, false, true);
  // TX 설정
  Radio.SetTxConfig(MODEM_LORA, 14, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, false, true, false, 0, false, 3000);

  Serial.println("╔══════════════════════════════════╗");
  Serial.println("║  AUV Base Station Ready          ║");
  Serial.println("║  Commands:                       ║");
  Serial.println("║    RUNNING   → start mission     ║");
  Serial.println("║    EMERGENCY → force ascent      ║");
  Serial.println("║    STATUS    → request status    ║");
  Serial.println("╚══════════════════════════════════╝");
  Radio.Rx(0);
}
 
void loop() {
  // Serial 입력 처리
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toUpperCase();

    if (input == "RUNNING") {
      Serial.println("→ Sending RUNNING command...");
      sendCommand(CMD_RUNNING);
    }
    else if (input == "EMERGENCY") {
      Serial.println("→ Sending EMERGENCY command...");
      sendCommand(CMD_EMERGENCY);
    }
    else if (input == "STATUS") {
      Serial.println("→ Sending STATUS request...");
      sendCommand(CMD_STATUS);
    }
    else {
      Serial.printf("Unknown: '%s' (try RUNNING / EMERGENCY / STATUS)\n",
                    input.c_str());
    }
  }

  // RX 처리 (인터럽트에서 벗어난 메인 루프에서 실행)
  if (rxPending) {
    rxPending = false;
    Serial.println("=================================");
    Serial.printf("📡 [Rx] %d bytes | RSSI: %d dBm | SNR: %d\n", rxSize, rxRssi, rxSnr);

    if (rxSize == sizeof(LoRaInsightPacket_t)) {
      LoRaInsightPacket_t* pkt = (LoRaInsightPacket_t*)rxBuf;
      Serial.println("┌─── AUV Insight ─────────────────────┐");
      Serial.printf("│ Time:      %lu s\n",   pkt->timestamp);
      Serial.printf("│ Dive #:    %d\n",      pkt->diveCount);
      Serial.printf("│ Max Depth: %.1f m\n",  pkt->maxDepth_dm * 0.1f);
      Serial.printf("│ Temp:      %d°C / %d°C\n",
                    pkt->tempSurface_c, pkt->tempBottom_c);
      Serial.printf("│ pH:        %.1f\n",    pkt->pH_mean_x10 * 0.1f);
      Serial.printf("│ DO:        %.1f mg/L\n", pkt->DO_mean_x10 * 0.1f);
      Serial.printf("│ EC:        %d μS/cm\n", pkt->EC_mean);
      Serial.printf("│ Anomalies: %d\n",      pkt->anomalyCount);
      Serial.printf("│ Alert:     [%s]\n",
                    alertNames[pkt->alertLevel & 0x03]);
      Serial.println("└─────────────────────────────────────┘");
    } else {
      Serial.printf("⚠️  Unknown packet (%d bytes): ", rxSize);
      for (int i = 0; i < rxSize && i < 16; i++)
        Serial.printf("%02X ", rxBuf[i]);
      Serial.println();
    }
  }

  Radio.IrqProcess();
}
