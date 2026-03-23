#pragma once
#include "stm32f4xx_hal.h"
#include "main.h"
#include "actuators/linear_actuator.h"
#include "drivers/l298n_stepper.h"

typedef enum {
    MOT_IDLE = 0,
    MOT_DIVE,
    MOT_CLIMB,
    MOT_SURFACE,
    MOT_ABORT
} MotionState;

typedef struct {
    MotionState state;
    uint32_t state_start_ms;

    // Targets (to be tuned later)
    float min_depth_m;
    float max_depth_m;

    // Actuator timing fallback (if no depth sensor yet)
    uint32_t dive_actuate_ms;
    uint32_t climb_actuate_ms;

    bool started;
} MotionCtx;

void Motion_Init(MotionCtx* m);
void Motion_Start(MotionCtx* m, uint32_t now_ms);
void Motion_Update(MotionCtx* m, uint32_t now_ms, GliderState* s,
                   LinearActuator* buoy, LinearActuator* mass, L298nStepper* stepper);
