#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>

// 📡 LoRa Settings
#define RF_FREQUENCY          868000000 
#define TX_OUTPUT_POWER       14        
#define LORA_BANDWIDTH        0         
#define LORA_SPREADING_FACTOR 7         
#define LORA_CODINGRATE       1         
#define LORA_PREAMBLE_LENGTH  8         

static RadioEvents_t RadioEvents;
HardwareSerial SerialSTM(2);

// 🎛️ Sensor Configurations
const int phPin = 1;        
const int ecPin = 4;        
const int analogDoPin = 5;  
const int DO_I2C_ADDR = 0x61; 

// 📊 평균 산출을 위한 샘플링 배열 및 변수
const int NUM_SAMPLES = 20;
int sampleCount = 0;
float phSamples[NUM_SAMPLES];
float ecSamples[NUM_SAMPLES];
float aDoSamples[NUM_SAMPLES];
float i2cO2Samples[NUM_SAMPLES];

// STM32에서 받아올 최신 수심/수온 데이터
float latestDepth = 0.0;
float latestTemp = 0.0;
bool stmConnected = false;

// ==========================================
// 🧮 수학 함수: 정렬(Bubble Sort) 및 절사평균(Trimmed Mean)
// ==========================================
void sortArray(float* arr, int size) {
  for (int i = 0; i < size - 1; ++i) {
    for (int j = 0; j < size - i - 1; ++j) {
      if (arr[j] > arr[j + 1]) {
        float tmp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = tmp;
      }
    }
  }
}

float getTrimmedMean(float* arr, int size, int trimCount) {
  sortArray(arr, size);
  float sum = 0;
  int cnt = 0;
  for (int i = trimCount; i < size - trimCount; ++i) {
    sum += arr[i];
    cnt++;
  }
  return sum / cnt; 
}
// ==========================================

void OnTxDone(void) { Serial.println("[LoRa] Tx Success! Data sent.\n"); }
void OnTxTimeout(void) { Serial.println("[LoRa] Tx Timeout! Failed.\n"); }

void setup() {
  Serial.begin(115200);
  SerialSTM.begin(115200, SERIAL_8N1, 18, 17);
  Mcu.begin(0, 0); 

  pinMode(45, OUTPUT);
  digitalWrite(45, LOW); 
  delay(100);
  
  Wire.begin(41, 42); 
  analogReadResolution(12); 

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, false,
                    true, 0, 0, false, 3000);

  Serial.println("\n--- ESP32 Ready (20-Sample Average Mode) ---");
}

void loop() {
  static uint32_t lastSendTime = 0;

  // 📥 1. STM32에서 들어오는 최신 수심 데이터 상시 수신 (1초에도 여러 번 들어올 수 있음)
  while (SerialSTM.available()) {
    String incomingMsg = SerialSTM.readStringUntil('\n');
    incomingMsg.trim();
    if (incomingMsg.startsWith("D:")) {
      int commaIndex = incomingMsg.indexOf(',');
      if (commaIndex > 0) {
        latestDepth = incomingMsg.substring(2, commaIndex).toFloat();
        latestTemp = incomingMsg.substring(commaIndex + 3).toFloat();
        stmConnected = true;
      }
    }
  }

  // 📡 2. 3초마다 ESP32 센서 읽기
  if (millis() - lastSendTime > 3000) {
    lastSendTime = millis(); 
    
    // 배열에 1개씩 데이터 쌓기
    phSamples[sampleCount] = analogRead(phPin) * (3.3 / 4095.0);
    ecSamples[sampleCount] = analogRead(ecPin) * (3.3 / 4095.0);
    aDoSamples[sampleCount] = analogRead(analogDoPin) * (3.3 / 4095.0);

    // I2C 산소 센서 읽기
    Wire.beginTransmission(DO_I2C_ADDR);
    Wire.write('R'); 
    Wire.endTransmission();
    delay(600); 
    Wire.requestFrom(DO_I2C_ADDR, 20);
    byte code = Wire.read();
    char do_data[20] = "";
    int i = 0;
    while (Wire.available() && i < 19) {
      do_data[i++] = Wire.read();
    }
    do_data[i] = '\0'; 

    if (code == 1) { 
      i2cO2Samples[sampleCount] = String(do_data).toFloat(); 
    } else { 
      i2cO2Samples[sampleCount] = -1.0; 
    }

    sampleCount++;
    Serial.print("Data Collected: "); Serial.print(sampleCount); Serial.println("/20");

    // =========================================================
    // 📈 3. 20개(1분치)가 모두 모였을 때 -> 평균 내서 전송!
    // =========================================================
    if (sampleCount >= NUM_SAMPLES) {
      
      // 위아래 2개씩 노이즈를 자르고(TrimCount=2) 평균 계산
      float avgPh = getTrimmedMean(phSamples, NUM_SAMPLES, 2);
      float avgEc = getTrimmedMean(ecSamples, NUM_SAMPLES, 2);
      float avgADo = getTrimmedMean(aDoSamples, NUM_SAMPLES, 2);
      float avgI2cO2 = getTrimmedMean(i2cO2Samples, NUM_SAMPLES, 2);

      // 평균값으로 Payload 완성! (STM32의 최신 수심값 포함)
      String dataMsg = "D:" + String(latestDepth) + ",T:" + String(latestTemp) + 
                       ",PH:" + String(avgPh) + ",EC:" + String(avgEc) + 
                       ",aDO:" + String(avgADo) + ",O2:" + String(avgI2cO2);
      
      Serial.println("\n=========================================");
      Serial.println("[20-Sample Averaged Payload]");
      Serial.println(dataMsg);
      
      // 🚀 Action A: LoRa로 기지국에 쏘기
      Radio.Send((uint8_t *)dataMsg.c_str(), dataMsg.length());
      
      // 💾 Action B: STM32로 넘겨서 SD카드에 저장하라고 명령하기!
      // 앞에 "SDLOG:" 라는 암호를 붙여서 보내면 STM32가 알아먹기 쉽지.
      SerialSTM.println("SDLOG:" + dataMsg); 
      
      Serial.println("=========================================\n");

      // 배열 카운트 초기화 (다음 1분을 위해)
      sampleCount = 0;
    }
  }

  // 필수: LoRa 백그라운드 프로세스
  Radio.IrqProcess(); 
}
