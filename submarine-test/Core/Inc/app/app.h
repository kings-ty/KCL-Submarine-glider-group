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

// Remote control (LoRa CMD:RUN / CMD:EMERGENCY)
void App_SetRunMode(uint8_t run);    // 1=Start cruise, 0=Stop
void App_EmergencyAscent(void);      // Emergency ascent

#ifdef __cplusplus
}
#endif