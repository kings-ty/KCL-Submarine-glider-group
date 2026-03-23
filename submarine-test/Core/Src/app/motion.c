/*
 * motion.c — Glider Motion Control
 *
 * PRINCIPLE OF MOTION  (trajectory P1-P7)
 * ----------------------------------------
 *  P1          : Deployed at surface. Positive buoyancy, glider floats.
 *  P1 -> P2    : Buoyancy reduced (syringe retracted -> negative buoyancy).
 *                Mass shifted forward toward nose -> positive pitch (nose down).
 *                Glider begins forward+downward motion.
 *  P2 -> P3    : Stable descending glide. Velocity, pitch, and angle of attack
 *                are constant. Net forces in equilibrium.
 *  P3 -> P5    : Attitude transition at depth. Buoyancy increased (syringe extended).
 *                Mass shifted toward tail -> negative pitch (nose up).
 *                Glider transitions descent->ascent. Lowest point reached at P4.
 *  P5 -> P6    : Stable ascending glide. Motion parameters constant.
 *  P6 -> P7    : Approaching surface. Mass returns toward centre. Pitch decreases.
 *                Glider surfaces and floats (positive buoyancy).
 *
 * PHYSICS SUMMARY
 * ---------------
 * Net buoyancy force:
 *   F_b = (rho * V - m_total) * g
 *   rho     = water density
 *   V       = glider displaced volume (changed by LD20 syringe)
 *   m_total = total glider mass
 *
 * Pitch torque from movable mass position:
 *   tau = m_mass * g * x_m * cos(theta)
 *   m_mass = 1 kg movable mass
 *   x_m    = position along body axis (controlled by L16-P)
 *   theta  = current pitch angle
 *
 * Lift and drag during steady glide:
 *   L = 0.5 * rho * v^2 * C_L(alpha) * A
 *   D = 0.5 * rho * v^2 * C_D(alpha) * A
 *   alpha = angle of attack
 *   v     = glide speed
 *   A     = wing reference area
 *
 * ACTUATOR -> PHYSICS MAPPING
 * ---------------------------
 *   LD20    (buoy)    : controls V (syringe volume)         -> buoyancy force F_b
 *   L16-P   (mass)    : controls x_m (mass along body axis) -> pitch torque tau
 *   Stepper (stepper) : rotates mass laterally              -> roll/heading control
 */

#include "app/motion.h"
#include "main.h" // send_log
#include <stdio.h>

static void set_state(MotionCtx* m, MotionState st, uint32_t now_ms)
{
    m->state = st;
    m->state_start_ms = now_ms;

    char msg[48];
    snprintf(msg, sizeof(msg), "[MOT] State=%d\r\n", (int)st);
    send_log(msg);
}

void Motion_Init(MotionCtx* m)
{
    m->state = MOT_IDLE;
    m->state_start_ms = 0;
    m->min_depth_m = 1.0f;
    m->max_depth_m = 10.0f;
    m->dive_actuate_ms = 4000;
    m->climb_actuate_ms = 4000;
    m->started = false;
}

void Motion_Start(MotionCtx* m, uint32_t now_ms)
{
    m->started = true;
    set_state(m, MOT_DIVE, now_ms);
}

void Motion_Update(MotionCtx* m, uint32_t now_ms, GliderState* s,
                   LinearActuator* buoy, LinearActuator* mass, L298nStepper* stepper)
{
    if (!m->started) {
        if (s->is_motor_on) {
            Motion_Start(m, now_ms);
        } else {
            return;
        }
    }

    switch (m->state) {
    case MOT_DIVE:
        // Negative buoyancy: retract syringe (reduces displaced volume -> F_b negative)
        if (buoy->state == ACT_IDLE) {
            Act_Retract(buoy, now_ms);
        }
        // Shift mass forward toward nose: positive pitch -> nose down -> descent
        if (mass->state == ACT_IDLE) {
            Act_Extend(mass, now_ms);
        }
        // Stepper holds neutral: no lateral rotation during straight dive
        Stepper_Stop(stepper);

        if ((now_ms - m->state_start_ms) > m->dive_actuate_ms) {
            Act_Stop(buoy);
            Act_Stop(mass);
            set_state(m, MOT_CLIMB, now_ms);
        }
        break;

    case MOT_CLIMB:
        // Positive buoyancy: extend syringe (increases displaced volume -> F_b positive)
        if (buoy->state == ACT_IDLE) {
            Act_Extend(buoy, now_ms);
        }
        // Shift mass toward tail: negative pitch -> nose up -> ascent
        if (mass->state == ACT_IDLE) {
            Act_Retract(mass, now_ms);
        }
        // Stepper holds neutral
        Stepper_Stop(stepper);

        if ((now_ms - m->state_start_ms) > m->climb_actuate_ms) {
            Act_Stop(buoy);
            Act_Stop(mass);
            set_state(m, MOT_DIVE, now_ms);
        }
        break;

    case MOT_SURFACE:
        // Full positive buoyancy, mass centred
        if (buoy->state != ACT_EXTENDING) {
            Act_Extend(buoy, now_ms);
        }
        Act_Stop(mass);
        Stepper_Stop(stepper);
        break;

    case MOT_ABORT:
    default:
        Act_Stop(buoy);
        Act_Stop(mass);
        Stepper_Stop(stepper);
        break;
    }
}
