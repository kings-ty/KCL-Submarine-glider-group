#include "app/system_check.h"
#include "main.h" // send_log
#include <stdio.h>

static bool read_imu_id(I2C_HandleTypeDef* hi2c, uint8_t addr7, uint8_t reg, uint8_t expected, uint8_t* out_id)
{
    if (HAL_I2C_Mem_Read(hi2c, (addr7 << 1), reg, 1, out_id, 1, 100) == HAL_OK) {
        return (*out_id == expected);
    }
    return false;
}

static uint16_t adc_read_once(ADC_HandleTypeDef* hadc)
{
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(hadc);
        return 0;
    }
    uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return v;
}

void SystemCheck_Init(SystemCheckCtx* c)
{
    c->state = SYSCHK_START;
    c->done = false;
    c->ok = false;
    c->step_start_ms = 0;
    c->imu_addr = 0;
}

void SystemCheck_Update(SystemCheckCtx* c, uint32_t now_ms,
                        I2C_HandleTypeDef* hi2c,
                        ADC_HandleTypeDef* hadc)
{
    if (c->done) return;

    switch (c->state) {
    case SYSCHK_START:
        send_log("[SYS] Self-test start\r\n");
        c->step_start_ms = now_ms;
        c->state = SYSCHK_I2C_SCAN;
        break;

    case SYSCHK_I2C_SCAN: {
        uint8_t id = 0;
        // Check BNO055
        if (read_imu_id(hi2c, 0x28, 0x00, 0xA0, &id)) {
            c->imu_addr = 0x28;
            send_log("[SYS] IMU: BNO055 @0x28\r\n");
            c->state = SYSCHK_ADC_READ;
        } else if (read_imu_id(hi2c, 0x29, 0x00, 0xA0, &id)) {
            c->imu_addr = 0x29;
            send_log("[SYS] IMU: BNO055 @0x29\r\n");
            c->state = SYSCHK_ADC_READ;
        } 
        // Check MPU9250
        else if (read_imu_id(hi2c, 0x69, 0x75, 0x71, &id)) {
            c->imu_addr = 0x69;
            send_log("[SYS] IMU: MPU9250 @0x69\r\n");
            c->state = SYSCHK_ADC_READ;
        } else if (read_imu_id(hi2c, 0x68, 0x75, 0x71, &id)) {
            c->imu_addr = 0x68;
            send_log("[SYS] IMU: MPU9250 @0x68\r\n");
            c->state = SYSCHK_ADC_READ;
        }
        else {
            send_log("[SYS] IMU not found (BNO055/MPU9250)\r\n");
            c->state = SYSCHK_FAIL;
        }
        break;
    }

    case SYSCHK_ADC_READ: {
        uint16_t v = adc_read_once(hadc);
        char msg[64];
        snprintf(msg, sizeof(msg), "[SYS] ADC raw: %u\r\n", v);
        send_log(msg);

        // Basic sanity: should not be stuck at 0 or 4095
        if (v == 0 || v >= 4095) {
            send_log("[SYS] ADC looks invalid\r\n");
            c->state = SYSCHK_FAIL;
        } else {
            c->state = SYSCHK_DONE;
        }
        break;
    }

    case SYSCHK_DONE:
        c->done = true;
        c->ok = true;
        send_log("[SYS] Self-test OK\r\n");
        break;

    case SYSCHK_FAIL:
    default:
        c->done = true;
        c->ok = false;
        send_log("[SYS] Self-test FAIL\r\n");
        break;
    }
}