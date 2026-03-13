#ifndef ESP32_COMM_H
#define ESP32_COMM_H

#include "stm32f4xx_hal.h"  // Adjust to your board's HAL (e.g., stm32g4xx_hal.h)
#include <stdbool.h>

// Function prototypes
void ESP32_Comm_Init(UART_HandleTypeDef *huart);
void ESP32_Comm_Process(void);
bool ESP32_Parse_Data(const char* data);

#endif /* ESP32_COMM_H */
