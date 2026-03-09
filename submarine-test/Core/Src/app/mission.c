#include "app/mission.h"
#include "main.h" // send_log
#include <stdio.h>

static void set_state(MissionCtx* m, MissionState st, uint32_t now_ms)
{
    m->state = st;
    m->state_start_ms = now_ms;

    char msg[48];
    snprintf(msg, sizeof(msg), "[MIS] State=%d\r\n", (int)st);
    send_log(msg);
}

void Mission_Init(MissionCtx* m)
{
    m->state = MIS_IDLE;
    m->state_start_ms = 0;
    m->min_depth_m = 1.0f;
    m->max_depth_m = 10.0f;
    m->dive_actuate_ms = 4000;
    m->climb_actuate_ms = 4000;
    m->started = false;
}

void Mission_Start(MissionCtx* m, uint32_t now_ms)
{
    m->started = true;
    set_state(m, MIS_DIVE, now_ms);
}

void Mission_Update(MissionCtx* m, uint32_t now_ms, GliderState* s, LinearActuator* buoy)
{
    // Temporary start condition: use serial command "CMD:MOTOR_ON" to start
    // main.c sets s->is_motor_on in process_serial_command().
    if (!m->started) {
        if (s->is_motor_on) {
            Mission_Start(m, now_ms);
        } else {
            return;
        }
    }

    switch (m->state) {
    case MIS_DIVE:
        // Negative buoyancy: retract syringe (direction may need swapping)
        // For now: retract for dive_actuate_ms then switch to climb.
        if (buoy->state == ACT_IDLE) {
            Act_Retract(buoy, now_ms);
        }

        if ((now_ms - m->state_start_ms) > m->dive_actuate_ms) {
            Act_Stop(buoy);
            set_state(m, MIS_CLIMB, now_ms);
        }
        break;

    case MIS_CLIMB:
        // Positive buoyancy: extend syringe
        if (buoy->state == ACT_IDLE) {
            Act_Extend(buoy, now_ms);
        }

        if ((now_ms - m->state_start_ms) > m->climb_actuate_ms) {
            Act_Stop(buoy);
            set_state(m, MIS_DIVE, now_ms);
        }
        break;

    case MIS_SURFACE:
        // Keep extended
        if (buoy->state != ACT_EXTENDING) {
            Act_Extend(buoy, now_ms);
        }
        break;

    case MIS_ABORT:
    default:
        // safest: stop and/or surface
        Act_Stop(buoy);
        break;
    }
}