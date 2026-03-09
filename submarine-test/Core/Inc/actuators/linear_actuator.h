#pragma once
#include <stdint.h>
#include "drivers/dfr0601_motor.h"

typedef enum { ACT_IDLE=0, ACT_EXTENDING, ACT_RETRACTING, ACT_FAULT } ActState;

typedef struct {
  DfrMotor motor;
  ActState state;

  uint16_t pwm_run;
  uint32_t safety_timeout_ms;
  uint32_t move_start_ms;

  // optional: position feedback for Actuonix L16-P
  uint8_t has_pos;
  ADC_HandleTypeDef* hadc;
  uint16_t target_adc;
  uint16_t deadband_adc;
} LinearActuator;

void Act_Init(LinearActuator* a, uint32_t now);
void Act_Extend(LinearActuator* a, uint32_t now);
void Act_Retract(LinearActuator* a, uint32_t now);
void Act_Stop(LinearActuator* a);
void Act_SetTargetADC(LinearActuator* a, uint16_t target_adc, uint32_t now);
void Act_Update(LinearActuator* a, uint32_t now);