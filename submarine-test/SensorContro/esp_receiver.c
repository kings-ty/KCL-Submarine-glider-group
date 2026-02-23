#include "LoRaWan_APP.h"
#include "Arduino.h"

// 📡 LoRa Frequency & Settings (Must match the sender)
#define RF_FREQUENCY          868000000 // 868MHz
#define LORA_BANDWIDTH        0         
#define LORA_SPREADING_FACTOR 7         
#define LORA_CODINGRATE       1         
#define LORA_PREAMBLE_LENGTH  8         

static RadioEvents_t RadioEvents;

// ⚡ Function triggered automatically on successful reception
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  String receivedText = "";
  for(int i = 0; i < size; i++) {
    receivedText += (char)payload[i];
  }

  // Print to Mac Mini Serial Monitor in English
  Serial.println("=================================");
  Serial.print("🎉 [Rx Success] Data: ");
  Serial.println(receivedText);
  Serial.print("Signal Strength(RSSI): ");
  Serial.print(rssi);
  Serial.println(" dBm");
  Serial.println("=================================");

  // Open ears for the next incoming data
  Radio.Rx(0); 
}

void setup() {
  Serial.begin(115200);

  // Initialize Board
  Mcu.begin();

  // Setup LoRa Radio
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    0, false, 0, true, false, 0, false, true);

  Serial.println("--- Base Station Receiver Started (Serial Only) ---");
  Radio.Rx(0); // Start listening
}

void loop() {
  // Keep the radio background process running
  Radio.IrqProcess(); 
}
