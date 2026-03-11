#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum { STEP_FWD = 1, STEP_REV = -1 } StepDir;

typedef struct {
    GPIO_TypeDef* in1_port; uint16_t in1_pin;
    GPIO_TypeDef* in2_port; uint16_t in2_pin;
    GPIO_TypeDef* in3_port; uint16_t in3_pin;
    GPIO_TypeDef* in4_port; uint16_t in4_pin;
    int8_t step_index;  // 0-3, current position in step table
} L298nStepper;

void Stepper_Init(L298nStepper* s);
void Stepper_Step(L298nStepper* s, StepDir dir);
void Stepper_Stop(L298nStepper* s);  // de-energise all coils
