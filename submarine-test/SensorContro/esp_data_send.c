#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "MS5837.h"

// 📡 LoRa Settings
#define RF_FREQUENCY          868000000 
#define TX_OUTPUT_POWER       14        
#define LORA_BANDWIDTH        0         
#define LORA_SPREADING_FACTOR 7         
#define LORA_CODINGRATE       1         
#define LORA_PREAMBLE_LENGTH  8         

static RadioEvents_t RadioEvents;

// 🎛️ Sensor Configurations
MS5837 depthSensor;
const int phPin = 1;        // Analog pH (Blue)
const int ecPin = 4;        // Analog EC (Black)
const int analogDoPin = 5;  // Analog DO (Black) - Kept as backup/comparison

const int DO_I2C_ADDR = 0x61; // New I2C Oxygen Sensor Address (97)

// LoRa Tx Events
void OnTxDone(void) {
  Serial.println("[LoRa] Tx Success! Data sent over the air.\n");
}
void OnTxTimeout(void) {
  Serial.println("[LoRa] Tx Timeout! Failed to send.\n");
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(); // Initialize Heltec board

  // 1. Enable Vext (Power for external sensors on V3)
  pinMode(45, OUTPUT);
  digitalWrite(45, LOW); 
  delay(100);
  
  // 2. Initialize I2C (SDA: 41, SCL: 42)
  // Both Depth Sensor and new Oxygen Sensor will share these pins
  Wire.begin(41, 42); 

  // 3. Initialize Depth Sensor
  if (!depthSensor.init()) {
    Serial.println("[Warning] Depth sensor not found! Check I2C wiring.");
  } else {
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.setFluidDensity(1025); // Set to 1025 for Seawater
    Serial.println("[OK] Depth sensor initialized.");
  }

  // 4. Set Analog Resolution to 12-bit (0~4095) for ESP32
  analogReadResolution(12); 

  // 5. Initialize LoRa Radio
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, false,
                    true, 0, 0, false, 3000);

  Serial.println("\n--- Submarine Transmitter Ready (Dual O2 Mode) ---");
}

void loop() {
  static uint32_t lastSendTime = 0;

  // Send data every 3 seconds (3000ms)
  if (millis() - lastSendTime > 3000) {
    
    // 1. Read Depth & Temperature
    depthSensor.read();
    float depth = depthSensor.depth();
    float waterTemp = depthSensor.temperature();

    // 2. Read Analog Sensors (Convert to Voltage)
    float phVolt = analogRead(phPin) * (3.3 / 4095.0);
    float ecVolt = analogRead(ecPin) * (3.3 / 4095.0);
    float aDoVolt = analogRead(analogDoPin) * (3.3 / 4095.0);

    // 3. Read New I2C Oxygen Sensor
    Wire.beginTransmission(DO_I2C_ADDR);
    Wire.write('R'); // Send 'Read' command
    Wire.endTransmission();
    
    delay(600); // Wait 600ms for the sensor to process

    Wire.requestFrom(DO_I2C_ADDR, 20);
    byte code = Wire.read();
    char do_data[20] = "";
    int i = 0;
    while (Wire.available() && i < 19) {
      do_data[i++] = Wire.read();
    }
    do_data[i] = '\0'; // Null-terminate the string

    String i2cO2Str = "Err";
    if (code == 1) {
      i2cO2Str = String(do_data);
    }

    // ==============================================
    // 🖨️ Print Detailed Logs in English
    // ==============================================
    Serial.println("=========================================");
    Serial.println("[Sensor Readings]");
    Serial.print("  Depth      : "); Serial.print(depth); Serial.println(" m");
    Serial.print("  Temp       : "); Serial.print(waterTemp); Serial.println(" C");
    Serial.print("  pH (Volt)  : "); Serial.print(phVolt); Serial.println(" V");
    Serial.print("  EC (Volt)  : "); Serial.print(ecVolt); Serial.println(" V");
    Serial.print("  Analog DO  : "); Serial.print(aDoVolt); Serial.println(" V");
    Serial.print("  I2C Oxygen : "); Serial.print(i2cO2Str); Serial.println(" mg/L");
    
    // 4. Create Payload String for Base Station
    // Used 'aDO' for analog and 'O2' for the new I2C sensor
    String dataMsg = "D:" + String(depth) + ",T:" + String(waterTemp) + 
                     ",PH:" + String(phVolt) + ",EC:" + String(ecVolt) + 
                     ",aDO:" + String(aDoVolt) + ",O2:" + i2cO2Str;
    
    Serial.print("[Tx Payload] => ");
    Serial.println(dataMsg);
    Serial.println("=========================================");

    // 5. Fire LoRa Radio!
    Radio.Send((uint8_t *)dataMsg.c_str(), dataMsg.length());
    
    lastSendTime = millis();
  }

  // Essential background process for LoRa radio
  Radio.IrqProcess(); 
}
