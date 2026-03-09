#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum {
    SYSCHK_START = 0,
    SYSCHK_I2C_SCAN,
    SYSCHK_IMU_ID,
    SYSCHK_ADC_READ,
    SYSCHK_DONE,
    SYSCHK_FAIL
} SysCheckState;

typedef struct {
    SysCheckState state;
    bool done;
    bool ok;
    uint32_t step_start_ms;
    uint8_t imu_addr;   // 0x28 or 0x29 if found
} SystemCheckCtx;

void SystemCheck_Init(SystemCheckCtx* c);
void SystemCheck_Update(SystemCheckCtx* c, uint32_t now_ms,
                        I2C_HandleTypeDef* hi2c,
                        ADC_HandleTypeDef* hadc);