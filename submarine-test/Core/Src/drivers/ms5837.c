#include "drivers/ms5837.h"

#define MS5837_ADDR             0xEC  // 0x76 << 1
#define MS5837_RESET            0x1E
#define MS5837_ADC_READ         0x00
#define MS5837_PROM_READ        0xA0
#define MS5837_CONVERT_D1_8192  0x4A
#define MS5837_CONVERT_D2_8192  0x5A

uint8_t MS5837_Init(MS5837_t *sensor, I2C_HandleTypeDef *hi2c) {
    sensor->hi2c = hi2c;
    sensor->fluidDensity = 1025.0f; // Default: Seawater (1025 kg/m3)

    // Reset sensor
    uint8_t cmd = MS5837_RESET;
    if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, &cmd, 1, 100) != HAL_OK) {
        return 0; // Sensor not found or error
    }
    HAL_Delay(10); // Wait for reset to complete

    // Read Calibration PROM (7 words)
    for (uint8_t i = 0; i < 7; i++) {
        cmd = MS5837_PROM_READ + (i * 2);
        if (HAL_I2C_Master_Transmit(hi2c, MS5837_ADDR, &cmd, 1, 100) != HAL_OK) return 0;
        
        uint8_t buf[2];
        if (HAL_I2C_Master_Receive(hi2c, MS5837_ADDR, buf, 2, 100) != HAL_OK) return 0;
        
        sensor->C[i] = (buf[0] << 8) | buf[1];
    }
    
    // Verify valid reading (C[1] must not be 0)
    if (sensor->C[1] == 0 || sensor->C[1] == 0xFFFF) {
        return 0;
    }

    return 1;
}

void MS5837_Read(MS5837_t *sensor) {
    uint8_t cmd;
    uint8_t buf[3];

    // 1. Request D1 (Pressure) conversion
    cmd = MS5837_CONVERT_D1_8192;
    HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_ADDR, &cmd, 1, 100);
    HAL_Delay(20); // Wait for conversion (~18-20ms)

    // Read D1 ADC value
    cmd = MS5837_ADC_READ;
    HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_ADDR, &cmd, 1, 100);
    HAL_I2C_Master_Receive(sensor->hi2c, MS5837_ADDR, buf, 3, 100);
    sensor->D1 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    // 2. Request D2 (Temperature) conversion
    cmd = MS5837_CONVERT_D2_8192;
    HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_ADDR, &cmd, 1, 100);
    HAL_Delay(20); // Wait for conversion

    // Read D2 ADC value
    cmd = MS5837_ADC_READ;
    HAL_I2C_Master_Transmit(sensor->hi2c, MS5837_ADDR, &cmd, 1, 100);
    HAL_I2C_Master_Receive(sensor->hi2c, MS5837_ADDR, buf, 3, 100);
    sensor->D2 = (buf[0] << 16) | (buf[1] << 8) | buf[2];

    // 3. Compute compensated temperature and pressure (MS5837-30BA math)
    int32_t dT;
    int64_t SENS, OFF;
    int32_t SENSi, OFFi, Ti;
    int64_t OFF2, SENS2;

    dT = (int32_t)sensor->D2 - ((int32_t)sensor->C[5] * 256);
    sensor->TEMP = 2000 + ((int64_t)dT * sensor->C[6]) / 8388608;

    SENS = ((int64_t)sensor->C[1] * 32768) + (((int64_t)sensor->C[3] * dT) / 256);
    OFF = ((int64_t)sensor->C[2] * 65536) + (((int64_t)sensor->C[4] * dT) / 128);

    // Second order compensation
    if (sensor->TEMP < 2000) {
        Ti = (3 * (int64_t)dT * dT) / 8589934592LL;
        OFFi = (3 * (sensor->TEMP - 2000) * (sensor->TEMP - 2000)) / 2;
        SENSi = (5 * (sensor->TEMP - 2000) * (sensor->TEMP - 2000)) / 8;
        if (sensor->TEMP < -1500) {
            OFFi += 7 * (sensor->TEMP + 1500) * (sensor->TEMP + 1500);
            SENSi += 4 * (sensor->TEMP + 1500) * (sensor->TEMP + 1500);
        }
    } else {
        Ti = (2 * (int64_t)dT * dT) / 137438953472LL;
        OFFi = (1 * (sensor->TEMP - 2000) * (sensor->TEMP - 2000)) / 16;
        SENSi = 0;
    }

    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;

    sensor->TEMP = sensor->TEMP - Ti;
    sensor->P = (((sensor->D1 * SENS2) / 2097152) - OFF2) / 8192;

    // 4. Save results to human-readable float fields
    sensor->temperature_c = sensor->TEMP / 100.0f;
    
    // P is in mbar * 10 (e.g. 1000mbar = 10000P for 02BA, for 30BA it's usually P/10 = mbar)
    float pressure_pa = (sensor->P / 10.0f) * 100.0f; 
    
    // Depth (m) = (Pressure(Pa) - 101300) / (density * 9.80665)
    sensor->depth_m = (pressure_pa - 101300.0f) / (sensor->fluidDensity * 9.80665f);
}
