#pragma once
#include "stm32f4xx_hal.h"
#include "main.h"
#include "actuators/linear_actuator.h"

typedef enum {
    MIS_IDLE = 0,
    MIS_DIVE,
    MIS_CLIMB,
    MIS_SURFACE,
    MIS_ABORT
} MissionState;

typedef struct {
    MissionState state;
    uint32_t state_start_ms;

    // Targets (to be tuned later)
    float min_depth_m;
    float max_depth_m;

    // Actuator timing fallback (if no depth sensor yet)
    uint32_t dive_actuate_ms;
    uint32_t climb_actuate_ms;

    bool started;
} MissionCtx;

void Mission_Init(MissionCtx* m);
void Mission_Start(MissionCtx* m, uint32_t now_ms);
void Mission_Update(MissionCtx* m, uint32_t now_ms, GliderState* s, LinearActuator* buoy);