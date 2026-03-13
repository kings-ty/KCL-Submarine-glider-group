#include "app/demo.h"
#include "main.h"   // send_log()
#include <stdio.h>

// Duration of each step in milliseconds
#define EXTEND_BUOY_MS    12000
#define RETRACT_BUOY_MS   12000
#define EXTEND_MASS_MS    2000
#define RETRACT_MASS_MS   2000
#define PAUSE_MS          1000

static void advance(DemoCtx* d, uint32_t now_ms)
{
    d->step = (DemoStep)((d->step + 1) % 8);
    d->step_start_ms = now_ms;

    char msg[48];
    snprintf(msg, sizeof(msg), "[DEMO] Step %d\r\n", (int)d->step);
    send_log(msg);
}

void Demo_Init(DemoCtx* d, uint32_t now_ms)
{
    d->step          = DEMO_EXTEND_BUOY;
    d->step_start_ms = now_ms;
    send_log("[DEMO] Starting actuator demo\r\n");
}

void Demo_Update(DemoCtx* d, uint32_t now_ms, LinearActuator* buoy, LinearActuator* mass)
{
    uint32_t elapsed = now_ms - d->step_start_ms;

    switch (d->step) {

    case DEMO_EXTEND_BUOY:
        if (buoy->state == ACT_IDLE) {
            send_log("[DEMO] Buoy EXTEND\r\n");
            Act_Extend(buoy, now_ms);
        }
        if (elapsed >= EXTEND_BUOY_MS) {
            Act_Stop(buoy);
            advance(d, now_ms);
        }
        break;

    case DEMO_PAUSE_1:
        if (elapsed >= PAUSE_MS) advance(d, now_ms);
        break;

    case DEMO_RETRACT_BUOY:
        if (buoy->state == ACT_IDLE) {
            send_log("[DEMO] Buoy RETRACT\r\n");
            Act_Retract(buoy, now_ms);
        }
        if (elapsed >= RETRACT_BUOY_MS) {
            Act_Stop(buoy);
            advance(d, now_ms);
        }
        break;

    case DEMO_PAUSE_2:
        if (elapsed >= PAUSE_MS) advance(d, now_ms);
        break;

    case DEMO_EXTEND_MASS:
        if (mass->state == ACT_IDLE) {
            send_log("[DEMO] Mass EXTEND\r\n");
            Act_Extend(mass, now_ms);
        }
        if (elapsed >= EXTEND_MASS_MS) {
            Act_Stop(mass);
            advance(d, now_ms);
        }
        break;

    case DEMO_PAUSE_3:
        if (elapsed >= PAUSE_MS) advance(d, now_ms);
        break;

    case DEMO_RETRACT_MASS:
        if (mass->state == ACT_IDLE) {
            send_log("[DEMO] Mass RETRACT\r\n");
            Act_Retract(mass, now_ms);
        }
        if (elapsed >= RETRACT_MASS_MS) {
            Act_Stop(mass);
            advance(d, now_ms);
        }
        break;

    case DEMO_PAUSE_4:
        if (elapsed >= PAUSE_MS) {
            send_log("[DEMO] Cycle complete, repeating\r\n");
            Act_Stop(buoy);
            Act_Stop(mass);
            advance(d, now_ms);
        }
        break;
    }
}
