#pragma once
#include "stm32f4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t C[8];
    uint32_t D1, D2;
    int32_t TEMP;
    int32_t P;
    float fluidDensity;
    float depth_m;
    float temperature_c;
} MS5837_t;

// Returns 1 on success, 0 on failure
uint8_t MS5837_Init(MS5837_t *sensor, I2C_HandleTypeDef *hi2c);

// Reads temperature and pressure, updates depth_m and temperature_c
void MS5837_Read(MS5837_t *sensor);
