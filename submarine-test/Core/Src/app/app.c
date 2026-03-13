#include "app/app.h"
#include "app/demo.h"
#include "drivers/l298n_stepper.h"
#include <stdio.h>
#include <string.h>

// Set to 1 to run the bench-test actuator demo (auto-starts, no serial command needed).
// Set to 0 for normal mission mode (send "CMD:MOTOR_ON" via USB serial to start).
#define DEMO_MODE  1

// Handles created in main.c by CubeMX
extern ADC_HandleTypeDef hadc1;   // PA0 - O2 sensor (ADC1_IN0)
extern ADC_HandleTypeDef hadc2;   // PA1 - LD20 buoyancy pot (ADC2_IN1)
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;

// App state
static GliderState s_state;

// Actuators
static LinearActuator s_buoy;
static LinearActuator s_mass;

// Stepper (mass rotation, L298N on PB12-PB15)
static L298nStepper s_stepper;

// Subsystems
static SystemCheckCtx s_sys;
static MotionCtx s_motion;
static DemoCtx s_demo;

// Scheduling
static uint32_t s_last_fast = 0;
static uint32_t s_last_slow = 0;

GliderState* App_State(void)
{
    return &s_state;
}

static void App_LogTelemetry(void)
{
    char msg[192];
    snprintf(msg, sizeof(msg),
             "V:%.2f D:%.2f O2:%.2f St:%d ML:%d Motor:%d\r\n",
             s_state.sensors.voltage,
             s_state.sensors.depth,
             s_state.sensors.o2,
             s_state.status,
             s_state.tinyml_result,
             (int)s_state.is_motor_on);
    send_log(msg);
}

void App_Init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_state.status = 0;
    s_state.is_motor_on = false;

    uint32_t now = HAL_GetTick();

    // ----------------------------
    // Buoyancy actuator (DFR0601)
    // PWM  -> PA8  (TIM1_CH1)
    // INA  -> PB0
    // INB  -> PB1
    // ----------------------------
    s_buoy.motor.ina_port = GPIOB;
    s_buoy.motor.ina_pin  = GPIO_PIN_0;

    s_buoy.motor.inb_port = GPIOB;
    s_buoy.motor.inb_pin  = GPIO_PIN_1;

    s_buoy.motor.htim_pwm    = &htim1;
    s_buoy.motor.pwm_channel = TIM_CHANNEL_1;
    s_buoy.motor.pwm_max     = 1000;

    s_buoy.pwm_run = 600;
    s_buoy.safety_timeout_ms = 15000;
    // LD20 has a potentiometer: Blue wire -> PA1 (ADC2_IN1)
    // Yellow -> 3.3V, White -> GND (not STM32 pins, use power header)
    // In DEMO_MODE run purely timed (no position feedback fighting the demo steps).
    // Set has_pos=1 and hadc in normal mission mode when target_adc is properly set.
#if DEMO_MODE
    s_buoy.has_pos = 0;
    s_buoy.hadc    = NULL;
#else
    s_buoy.has_pos = 1;
    s_buoy.hadc    = &hadc2;
#endif
    Act_Init(&s_buoy, now);

    // ----------------------------
    // Mass actuator (DFR0601)
    // PWM  -> PA9  (TIM1_CH2)
    // INA  -> PB10
    // INB  -> PB11
    // ----------------------------
    s_mass.motor.ina_port = GPIOB;
    s_mass.motor.ina_pin  = GPIO_PIN_10;

    s_mass.motor.inb_port = GPIOB;
    s_mass.motor.inb_pin  = GPIO_PIN_11;

    s_mass.motor.htim_pwm    = &htim1;
    s_mass.motor.pwm_channel = TIM_CHANNEL_2;
    s_mass.motor.pwm_max     = 1000;

    s_mass.pwm_run = 500;
    s_mass.safety_timeout_ms = 8000;

    // As Actuonix is L16-P, connect its feedback to an ADC channel.
    s_mass.has_pos = 1;
    s_mass.hadc = NULL;
    Act_Init(&s_mass, now);

    // Stepper motor (L298N, mass rotation)
    // IN1=PB12, IN2=PB13, IN3=PB14, IN4=PB15
    s_stepper.in1_port = GPIOB; s_stepper.in1_pin = GPIO_PIN_12;
    s_stepper.in2_port = GPIOB; s_stepper.in2_pin = GPIO_PIN_13;
    s_stepper.in3_port = GPIOB; s_stepper.in3_pin = GPIO_PIN_14;
    s_stepper.in4_port = GPIOB; s_stepper.in4_pin = GPIO_PIN_15;
    Stepper_Init(&s_stepper);

    // System check + mission/demo
    SystemCheck_Init(&s_sys);
    Motion_Init(&s_motion);
#if DEMO_MODE
    Demo_Init(&s_demo, now);
    send_log("[APP] Init complete (DEMO MODE)\r\n");
#else
    send_log("[APP] Init complete\r\n");
#endif
}

void App_Tick(void)
{
    uint32_t now = HAL_GetTick();

    // Fast tasks @ 20ms
    if ((now - s_last_fast) >= 20) {
        s_last_fast = now;

        Act_Update(&s_buoy, now);
        Act_Update(&s_mass, now);

#if DEMO_MODE
        Demo_Update(&s_demo, now, &s_buoy, &s_mass, &s_stepper);
#else
        if (!s_sys.done) {
            SystemCheck_Update(&s_sys, now, &hi2c1, &hadc1);
        } else {
            Motion_Update(&s_motion, now, &s_state, &s_buoy, &s_mass, &s_stepper);
        }
#endif
    }

    // Slow tasks @ 1000ms
    if ((now - s_last_slow) >= 1000) {
        s_last_slow = now;

        get_simulated_sensors(&s_state.sensors);
        s_state.status = rand() % 3;
        s_state.tinyml_result = rand() % 2;

        App_LogTelemetry();
    }
}