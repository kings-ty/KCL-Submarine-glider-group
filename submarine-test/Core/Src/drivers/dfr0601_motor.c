#include "drivers/dfr0601_motor.h"

static void pwm_set(DfrMotor* m, uint16_t pwm){
  if (pwm > m->pwm_max) pwm = m->pwm_max;
  __HAL_TIM_SET_COMPARE(m->htim_pwm, m->pwm_channel, pwm);
}

void DfrMotor_Init(DfrMotor* m){
  HAL_TIM_PWM_Start(m->htim_pwm, m->pwm_channel);
  DfrMotor_Stop(m);
}

void DfrMotor_Stop(DfrMotor* m){
  pwm_set(m, 0);
  HAL_GPIO_WritePin(m->ina_port, m->ina_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(m->inb_port, m->inb_pin, GPIO_PIN_RESET);
}

void DfrMotor_Set(DfrMotor* m, DfrDir dir, uint16_t pwm){
  switch(dir){
    case DFR_FWD:
      HAL_GPIO_WritePin(m->ina_port, m->ina_pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(m->inb_port, m->inb_pin, GPIO_PIN_SET);
      pwm_set(m, pwm);
      break;
    case DFR_REV:
      HAL_GPIO_WritePin(m->ina_port, m->ina_pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(m->inb_port, m->inb_pin, GPIO_PIN_RESET);
      pwm_set(m, pwm);
      break;
    default:
      DfrMotor_Stop(m);
      break;
  }
}