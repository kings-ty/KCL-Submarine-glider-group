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
#define NUM_SAMPLES 20

HardwareSerial SerialSTM(2);
static RadioEvents_t RadioEvents;

float depthSamples[NUM_SAMPLES];
float tempSamples[NUM_SAMPLES];
float phSamples[NUM_SAMPLES];
float ecSamples[NUM_SAMPLES];
float aDoSamples[NUM_SAMPLES];
float i2cO2Samples[NUM_SAMPLES];

int sampleCount = 0; // One time Colleting sample count
unsigned long lastReadTime = 0;
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
  SerialSTM.begin(115200, SERIAL_8n1, 18, 17);
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
//Sort Array(bubble sort)
void sortArray(float* arr, int size){
  for(int i =0; i < size - 1; ++i){
    for(int j=0; j < size -i-1; ++j){
      if(arr[j] > arr[j+1]){
        float tmp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = tmp;
      }
    }
  }
}
// except for n, make an mean(Trimmed Mean)
float getTrimmedMean(float* arr, int size, int trimCount){
  sortArray(arr, size);
  float sum = 0;
  int cnt = 0;
  // top and bottom to sum
  for(int i=trimCount; i<size-trimCount; ++i){
    sum += arr[i];
    cnt++;
  }
  return sum / cnt //Average
}
//Flag of Depth for Lora
bool allowLoRaTx = false;

void loop() {
  static uint32_t lastSendTime = 0;

  //-------------------
  // STM DATA RECEIVER
  //-------------------
  if(SerialSTM.available()){
    //STM32 to read of \n
    String incomingMsg = SerialSTM.readStringUntil('\n');
    incomingMsg.trim();
    Serial.print("[The message come from STM32]:");
    Serial.println(incomingMsg);
    if (incomingMsg == "SEND_OK"){
      allowLoRaTx = true;
      Serial.println("👉 LoRa permission!");
    }else if(incomingMsg == "SEND_NO"){
      allowLoRaTx = false;
      Serial.println("👉 LoRa banned!");
    }
  }
  // Send data every 3 seconds (3000ms)
  if (millis() - lastSendTime > 3000) {
    lastReadTime = millis();
    // 1. Read Depth & Temperature
    depthSensor.read();
    depthSamples[sampleCount] = depthSensor.depth();
    tempSamples[sampleCount] = depthSensor.temperature();
    phSamples[sampleCount] = analogRead(phPin) * (3.3 / 4095.0);
    ecSamples[sampleCount] = analogRead(ecPin) * (3.3 / 4095.0);
    aDoSamples[sampleCount] = analogRead(analogDoPin) * (3.3 / 4095.0);
    // float depth = depthSensor.depth();
    // float waterTemp = depthSensor.temperature();

    // // 2. Read Analog Sensors (Convert to Voltage)
    // float phVolt = analogRead(phPin) * (3.3 / 4095.0);
    // float ecVolt = analogRead(ecPin) * (3.3 / 4095.0);
    // float aDoVolt = analogRead(analogDoPin) * (3.3 / 4095.0);

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

    if (code == 1) {
      i2cO2Samples[sampleCount] = String(do_data).toFloat();
      //Convert string to float
    } else{
      //Error occured -1
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
