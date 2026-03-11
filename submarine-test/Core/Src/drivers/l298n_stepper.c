#include "drivers/l298n_stepper.h"

// Full-step bipolar sequence (4 steps)
// IN1 IN2 IN3 IN4
static const uint8_t STEP_TABLE[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1},
};

static void apply_step(L298nStepper* s)
{
    const uint8_t* row = STEP_TABLE[s->step_index];
    HAL_GPIO_WritePin(s->in1_port, s->in1_pin, row[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in2_port, s->in2_pin, row[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in3_port, s->in3_pin, row[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in4_port, s->in4_pin, row[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Stepper_Init(L298nStepper* s)
{
    s->step_index = 0;
    Stepper_Stop(s);
}

void Stepper_Step(L298nStepper* s, StepDir dir)
{
    s->step_index = (s->step_index + dir + 4) % 4;
    apply_step(s);
}

void Stepper_Stop(L298nStepper* s)
{
    HAL_GPIO_WritePin(s->in1_port, s->in1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in2_port, s->in2_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in3_port, s->in3_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->in4_port, s->in4_pin, GPIO_PIN_RESET);
}
