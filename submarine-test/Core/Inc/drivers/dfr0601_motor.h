#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum { DFR_STOP=0, DFR_FWD, DFR_REV } DfrDir;

typedef struct {
  GPIO_TypeDef* ina_port; uint16_t ina_pin;
  GPIO_TypeDef* inb_port; uint16_t inb_pin;
  TIM_HandleTypeDef* htim_pwm; uint32_t pwm_channel;
  uint16_t pwm_max;  // ARR value
} DfrMotor;

void DfrMotor_Init(DfrMotor* m);
void DfrMotor_Set(DfrMotor* m, DfrDir dir, uint16_t pwm);
void DfrMotor_Stop(DfrMotor* m);