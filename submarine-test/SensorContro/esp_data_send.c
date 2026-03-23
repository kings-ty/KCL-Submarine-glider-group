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

// 📊 Arrays and variables for averaging sensor samples
const int NUM_SAMPLES = 20;
int sampleCount = 0;
float phSamples[NUM_SAMPLES];
float ecSamples[NUM_SAMPLES];
float aDoSamples[NUM_SAMPLES];
float i2cO2Samples[NUM_SAMPLES];

// Latest depth/temperature data from STM32
float latestDepth = 0.0;
float latestTemp = 0.0;
bool stmConnected = false;

// ==========================================
// 🧮 Math functions: Bubble Sort and Trimmed Mean
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
  Serial.println("\n--- ESP32 Ready (20-Sample Average Mode) ---");
}

void loop() {
  static uint32_t lastSendTime = 0;

  // 📥 1. Continuously receive the latest depth data from STM32 (may arrive multiple times per second)
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

  // 📡 2. Read ESP32 sensors every 3 seconds
  if (millis() - lastSendTime > 3000) {
    lastSendTime = millis(); 
    
    // Store one sample in the arrays
    phSamples[sampleCount] = analogRead(phPin) * (3.3 / 4095.0);
    ecSamples[sampleCount] = analogRead(ecPin) * (3.3 / 4095.0);
    aDoSamples[sampleCount] = analogRead(analogDoPin) * (3.3 / 4095.0);

    // Read dissolved oxygen from I2C sensor
    Wire.beginTransmission(DO_I2C_ADDR);
    Wire.write('R'); 
    Wire.write('R'); 
    Wire.endTransmission();
    delay(600); 
    delay(600); 
    Wire.requestFrom(DO_I2C_ADDR, 20);
    byte code = Wire.read();
    char do_data[20] = "";
    int i = 0;
    while (Wire.available() && i < 19) {
      do_data[i++] = Wire.read();
    }
    do_data[i] = '\0'; 
    do_data[i] = '\0'; 

    if (code == 1) { 
      i2cO2Samples[sampleCount] = String(do_data).toFloat(); 
    } else { 
      i2cO2Samples[sampleCount] = -1.0; 
    if (code == 1) { 
      i2cO2Samples[sampleCount] = String(do_data).toFloat(); 
    } else { 
      i2cO2Samples[sampleCount] = -1.0; 
    }

    sampleCount++;
    Serial.print("Data Collected: "); Serial.print(sampleCount); Serial.println("/20");
    // ==============================================
    // 🖨️ Print Detailed Logs in English
    // ==============================================
    if (sampleCount >= NUM_SAMPLES){

    
    // 4. Create Payload String for Base Station
    // Used 'aDO' for analog and 'O2' for the new I2C sensor
    String dataMsg = "D:" + String(depth) + ",T:" + String(waterTemp) + 
                     ",PH:" + String(phVolt) + ",EC:" + String(ecVolt) + 
                     ",aDO:" + String(aDoVolt) + ",O2:" + i2cO2Str;
    
    Serial.print("[Tx Payload] => ");
    Serial.println("\n[1 Min Avg Payload] => " + dataMsg);
    // 5. Fire LoRa Radio!
    Radio.Send((uint8_t *)dataMsg.c_str(), dataMsg.length());
    // Fire STM32
    SerialSTM.println(dataMsg);
    sampleCount = 0;
    }
  }

  // Essential background process for LoRa radio
  Radio.IrqProcess(); 
}
