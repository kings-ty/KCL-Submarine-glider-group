#include "esp32_comm.h"
#include <string.h>
#include <stdlib.h>

#define RX_BUFFER_SIZE 128
#define DEPTH_THRESHOLD 1.0f  // 🌊 Depth threshold to allow LoRa tx (e.g., 1.0 meters)

static UART_HandleTypeDef *esp_uart;
static uint8_t rx_byte;
static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;
static bool data_ready = false;

// 1️⃣ Initialize communication (Call in main.c setup)
void ESP32_Comm_Init(UART_HandleTypeDef *huart) {
    esp_uart = huart;
    // Start UART interrupt to receive 1 byte at a time
    HAL_UART_Receive_IT(esp_uart, &rx_byte, 1);
}

// 2️⃣ Main process to run continuously in the while(1) loop
void ESP32_Comm_Process(void) {
    if (data_ready) {
        // Parse depth from ESP32 data and check the condition
        bool allow_tx = ESP32_Parse_Data(rx_buffer);

        // Send command (command + newline) to ESP32
        if (allow_tx) {
            char msg[] = "SEND_OK\n";
            HAL_UART_Transmit(esp_uart, (uint8_t*)msg, strlen(msg), 100);
        } else {
            char msg[] = "SEND_NO\n";
            HAL_UART_Transmit(esp_uart, (uint8_t*)msg, strlen(msg), 100);
        }

        // Clear buffer and reset for the next reception
        memset(rx_buffer, 0, RX_BUFFER_SIZE);
        rx_index = 0;
        data_ready = false;
    }
}

// 3️⃣ Data parsing function (Find "D:" and convert to float)
bool ESP32_Parse_Data(const char* data) {
    // Expected received data format: "D:1.23,T:20.5,PH:1.5,EC:2.1,aDO:1.1,O2:8.5"
    char *d_ptr = strstr(data, "D:"); // Find the starting position of "D:"
    
    if (d_ptr != NULL) {
        // Convert the characters after "D:" to a float
        float depth = atof(d_ptr + 2); 

        // Return true if depth is greater than or equal to the threshold
        if (depth >= DEPTH_THRESHOLD) {
            return true;
        }
    }
    // Return false if parsing fails or depth is too shallow
    return false; 
}

// 4️⃣ UART Rx Complete Callback (Executes per byte received)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == esp_uart->Instance) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Null-terminate the string
                data_ready = true;          // Set flag to start processing
            }
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = rx_byte; // Store character in buffer
            }
        }
        // Re-enable interrupt to receive the next byte
        HAL_UART_Receive_IT(esp_uart, &rx_byte, 1);
    }
}
