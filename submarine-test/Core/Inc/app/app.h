#pragma once

#include "stm32f4xx_hal.h"
#include "main.h"   // for GliderState, send_log()

#include "actuators/linear_actuator.h"
#include "app/system_check.h"
#include "app/motion.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Tick(void);

// Accessors (optional for now)
GliderState* App_State(void);

#ifdef __cplusplus
}
#endif