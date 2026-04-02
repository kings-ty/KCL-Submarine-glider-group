#include "app/esp32_comm.h"
#include "app/stm32_sdlog_receiver.h"
#include <string.h>
#include <stdlib.h>

#define DEPTH_THRESHOLD 1.0f  // 🌊 Depth threshold to allow LoRa tx (e.g., 1.0 meters)

static UART_HandleTypeDef *esp_uart;

// 1️⃣ Initialize communication (Call in main.c setup)
void ESP32_Comm_Init(UART_HandleTypeDef *huart) {
    esp_uart = huart;
}

// 2️⃣ Process Data (Called directly from main.c's Rx Callback)
void ESP32_Process_Data(const char* data) {
    // Save SDLOG data to SD card
    sdlog_parse_and_save(data);

    // Find the starting position of "D:"
    char *d_ptr = strstr(data, "D:"); 
    
    if (d_ptr != NULL) {
        // Convert the characters after "D:" to a float
        float depth = atof(d_ptr + 2); 

        // Check condition and send response to ESP32
        if (depth >= DEPTH_THRESHOLD) {
            char msg[] = "SEND_OK\n";
            HAL_UART_Transmit(esp_uart, (uint8_t*)msg, strlen(msg), 100);
        } else {
            char msg[] = "SEND_NO\n";
            HAL_UART_Transmit(esp_uart, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}
