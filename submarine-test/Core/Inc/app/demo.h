#pragma once

#include "stm32f4xx_hal.h"
#include "actuators/linear_actuator.h"
#include "drivers/l298n_stepper.h"

/**
 * Demo mode: runs a timed bench-test sequence that exercises both actuators
 * and the stepper motor without needing any serial command trigger.
 *
 * Sequence (repeating):
 *  1. Extend buoyancy actuator  (12 s)
 *  2. Stop, pause               (1 s)
 *  3. Retract buoyancy actuator (12 s)
 *  4. Stop, pause               (1 s)
 *  5. Extend mass actuator      (2 s)
 *  6. Stop, pause               (1 s)
 *  7. Retract mass actuator     (2 s)
 *  8. Stop, pause               (1 s)
 *  9. Stepper rotate forward    (2 s)
 * 10. Stop, pause               (1 s)
 * 11. Stepper rotate reverse    (2 s)
 * 12. Stop, pause               (1 s) -> back to step 1
 */

typedef enum {
    DEMO_EXTEND_BUOY = 0,
    DEMO_PAUSE_1,
    DEMO_RETRACT_BUOY,
    DEMO_PAUSE_2,
    DEMO_EXTEND_MASS,
    DEMO_PAUSE_3,
    DEMO_RETRACT_MASS,
    DEMO_PAUSE_4,
    DEMO_STEP_FWD,
    DEMO_PAUSE_5,
    DEMO_STEP_REV,
    DEMO_PAUSE_6
} DemoStep;

typedef struct {
    DemoStep step;
    uint32_t step_start_ms;
} DemoCtx;

void Demo_Init(DemoCtx* d, uint32_t now_ms);
void Demo_Update(DemoCtx* d, uint32_t now_ms, LinearActuator* buoy, LinearActuator* mass, L298nStepper* stepper);
