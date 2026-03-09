#include "actuators/linear_actuator.h"

static void fault(LinearActuator* a){
  a->state = ACT_FAULT;
  DfrMotor_Stop(&a->motor);
}

static uint16_t read_adc(ADC_HandleTypeDef* hadc){
  HAL_ADC_Start(hadc);
  if (HAL_ADC_PollForConversion(hadc, 5) != HAL_OK){
    HAL_ADC_Stop(hadc);
    return 0;
  }
  uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return v;
}

void Act_Init(LinearActuator* a, uint32_t now){
  DfrMotor_Init(&a->motor);
  a->state = ACT_IDLE;
  a->move_start_ms = now;
  if (a->pwm_run == 0) a->pwm_run = (uint16_t)(a->motor.pwm_max * 0.6f);
  if (a->safety_timeout_ms == 0) a->safety_timeout_ms = 12000;
  if (a->deadband_adc == 0) a->deadband_adc = 10;
}

void Act_Extend(LinearActuator* a, uint32_t now){
  a->state = ACT_EXTENDING;
  a->move_start_ms = now;
  DfrMotor_Set(&a->motor, DFR_FWD, a->pwm_run);
}

void Act_Retract(LinearActuator* a, uint32_t now){
  a->state = ACT_RETRACTING;
  a->move_start_ms = now;
  DfrMotor_Set(&a->motor, DFR_REV, a->pwm_run);
}

void Act_Stop(LinearActuator* a){
  a->state = ACT_IDLE;
  DfrMotor_Stop(&a->motor);
}

void Act_SetTargetADC(LinearActuator* a, uint16_t target_adc, uint32_t now){
  a->target_adc = target_adc;
  uint16_t pos = read_adc(a->hadc);
  if (pos == 0){ fault(a); return; }

  if (pos + a->deadband_adc < target_adc) Act_Extend(a, now);
  else if (pos > target_adc + a->deadband_adc) Act_Retract(a, now);
  else Act_Stop(a);
}

void Act_Update(LinearActuator* a, uint32_t now){
  if (a->state == ACT_IDLE || a->state == ACT_FAULT) return;

  if ((now - a->move_start_ms) > a->safety_timeout_ms){
    fault(a);
    return;
  }

  if (a->has_pos){
    uint16_t pos = read_adc(a->hadc);
    if (pos == 0){ fault(a); return; }

    if (pos + a->deadband_adc < a->target_adc){
      if (a->state != ACT_EXTENDING) Act_Extend(a, now);
    } else if (pos > a->target_adc + a->deadband_adc){
      if (a->state != ACT_RETRACTING) Act_Retract(a, now);
    } else {
      Act_Stop(a);
    }
  }
}