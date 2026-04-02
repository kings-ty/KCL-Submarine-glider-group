#ifndef STM32_SDLOG_RECEIVER_H
#define STM32_SDLOG_RECEIVER_H

#include "stm32f4xx_hal.h"

/* Emergency ascent flag - Read in while(1) for motor control */
extern volatile uint8_t g_emergencyAscent;
extern volatile uint8_t g_emergencyReason;

/* Public functions */
HAL_StatusTypeDef sdlog_init(void);                       /* Call after MX_FATFS_Init() */
void              sdlog_parse_and_save(const char* line); /* Call inside ESP32_Process_Data() */
void              sdlog_check_emergency_byte(uint8_t b);  /* Call for each UART3 callback byte */
void              sdlog_task(void);                       /* Call inside while(1) */

#endif /* STM32_SDLOG_RECEIVER_H */
