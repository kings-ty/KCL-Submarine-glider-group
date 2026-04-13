// #include "LoRaWan_APP.h"
// #include "Arduino.h"
// #include <Wire.h>

// // ============================================================
// // 📡 LoRa Settings (Strictly following user's spec)
// // ============================================================
// #define RF_FREQUENCY          868000000 
// #define TX_OUTPUT_POWER       14        
// #define LORA_BANDWIDTH        0         
// #define LORA_SPREADING_FACTOR 7         
// #define LORA_CODINGRATE       1         
// #define LORA_PREAMBLE_LENGTH  8         

// static RadioEvents_t RadioEvents;
// HardwareSerial SerialSTM(2);

// void OnTxDone(void) { Serial.println("[LoRa] Tx Success! Data sent.\n"); }
// void OnTxTimeout(void) { Serial.println("[LoRa] Tx Timeout! Failed.\n"); }

// void setup() {
//   // 1. Hardware Init (Following user's code pins and baudrates)
//   Serial.begin(115200);
//   SerialSTM.begin(115200, SERIAL_8N1, 18, 17);
//   Mcu.begin(0, 0); 

//   pinMode(45, OUTPUT);
//   digitalWrite(45, LOW); 
//   delay(100);
  
//   Wire.begin(41, 42); 
//   analogReadResolution(12); 

//   // 2. LoRa Radio Init (Following user's spec)
//   RadioEvents.TxDone = OnTxDone;
//   RadioEvents.TxTimeout = OnTxTimeout;
//   Radio.Init(&RadioEvents);
//   Radio.SetChannel(RF_FREQUENCY);
//   Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
//                     LORA_SPREADING_FACTOR, LORA_CODINGRATE,
//                     LORA_PREAMBLE_LENGTH, false,
//                     true, 0, 0, false, 3000);

//   Serial.println("\n--- ESP32 HIL Bridge Online (LoRa Lib Compliant) ---");
//   Serial.println("--- Waiting for Simulation Data from PC (Serial) ---");
// }

// void loop() {
//   // 📥 1. Receive simulated sample from Python Gateway via Serial (USB)
//   // Format: "D:0.00,T:25.00,PH:8.10,EC:50.000,aDO:8.10,O2:8.10"
//   if (Serial.available() > 0) {
//     String payload = Serial.readStringUntil('\n');
//     payload.trim();

//     if (payload.length() > 0) {
//       Serial.print("[HIL] Forwarding to LoRa: ");
//       Serial.println(payload);

//       // 📡 2. Send the exact payload over LoRa using user's library
//       Radio.Send((uint8_t *)payload.c_str(), payload.length());
      
//       // 3. Mirror to SerialSTM (STM32) as in user's original logic
//       SerialSTM.println(payload);
//     }
//   }

//   // Keep STM32 serial buffer clear
//   while (SerialSTM.available()) {
//     SerialSTM.read();
//   }

//   // Required for LoRa radio operation
//   Radio.IrqProcess(); 
// }
