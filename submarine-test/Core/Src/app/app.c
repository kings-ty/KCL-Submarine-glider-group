#include "app/app.h"
#include "app/demo.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "drivers/ms5837.h"

// Set to 1 to run the bench-test actuator demo (auto-starts, no serial command needed).
// Set to 0 for normal mission mode (send "CMD:MOTOR_ON" via USB serial to start).
#define DEMO_MODE  1

// Handles created in main.c by CubeMX
extern ADC_HandleTypeDef hadc1;   // PA0 - O2 sensor (ADC1_IN0)
//extern ADC_HandleTypeDef hadc2;   // PA1 - LD20 buoyancy pot (ADC2_IN1)
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;
// App state
static GliderState s_state;

// Actuators
static LinearActuator s_buoy;
static LinearActuator s_mass;

// Subsystems
static SystemCheckCtx s_sys;
static MissionCtx s_mission;
static DemoCtx s_demo;

// Depth Sensor
static MS5837_t s_depth_sensor;
static uint8_t s_depth_sensor_ok = 0;

// Scheduling
static uint32_t s_last_fast = 0;
static uint32_t s_last_slow = 0;

GliderState* App_State(void)
{
    return &s_state;
}

extern float yaw, roll, pitch;

static void App_LogTelemetry(void)
{
    char msg[256];
    int v_int = (int)s_state.sensors.voltage;
    int v_dec = (int)((s_state.sensors.voltage - v_int) * 100);
    if (v_dec < 0) v_dec = -v_dec;

    int d_int = (int)s_state.sensors.depth;
    int d_dec = (int)((s_state.sensors.depth - d_int) * 100);
    if (d_dec < 0) d_dec = -d_dec;

    int o2_int = (int)s_state.sensors.o2;
    int o2_dec = (int)((s_state.sensors.o2 - o2_int) * 100);
    if (o2_dec < 0) o2_dec = -o2_dec;

    int y_int = (int)yaw;
    int y_dec = (int)((yaw - y_int) * 100);
    if (y_dec < 0) y_dec = -y_dec;

    int r_int = (int)roll;
    int r_dec = (int)((roll - r_int) * 100);
    if (r_dec < 0) r_dec = -r_dec;

    int p_int = (int)pitch;
    int p_dec = (int)((pitch - p_int) * 100);
    if (p_dec < 0) p_dec = -p_dec;

    snprintf(msg, sizeof(msg),
             "V:%d.%02d D:%d.%02d O2:%d.%02d | Y:%d.%02d R:%d.%02d P:%d.%02d | St:%d ML:%d Motor:%d\r\n",
             v_int, v_dec,
             d_int, d_dec,
             o2_int, o2_dec,
             y_int, y_dec,
             r_int, r_dec,
             p_int, p_dec,
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
    s_buoy.safety_timeout_ms = 12000;
    // LD20 has a potentiometer: Blue wire -> PA1 (ADC2_IN1)
    // Yellow -> 3.3V, White -> GND (not STM32 pins, use power header)
    s_buoy.has_pos = 0;
    s_buoy.hadc = NULL;
    Act_Init(&s_buoy, now);

    // ----------------------------
    // Mass actuator (DFR0601)
    // PWM  -> PA9  (TIM1_CH2)
    // INA  -> PB10
    // INB  -> PB11
    // ----------------------------
    s_mass.motor.ina_port = GPIOE;
    s_mass.motor.ina_pin  = GPIO_PIN_9;

    s_mass.motor.inb_port = GPIOE;
    s_mass.motor.inb_pin  = GPIO_PIN_11;

    s_mass.motor.htim_pwm    = &htim1;
    s_mass.motor.pwm_channel = TIM_CHANNEL_2;
    s_mass.motor.pwm_max     = 1000;

    s_mass.pwm_run = 500;
    s_mass.safety_timeout_ms = 8000;

    // As Actuonix is L16-P, connect its feedback to an ADC channel.
    s_mass.has_pos = 0;
    s_mass.hadc = NULL;
    Act_Init(&s_mass, now);

    // System check + mission/demo
    SystemCheck_Init(&s_sys);
    Mission_Init(&s_mission);
    // Initialize MS5837 Depth Sensor on hi2c2
    s_depth_sensor_ok = MS5837_Init(&s_depth_sensor, &hi2c2);
    if(s_depth_sensor_ok) {
        send_log("[APP] MS5837 Depth sensor initialized successfully.\r\n");
    } else {
        send_log("[APP] MS5837 not found!\r\n");
    }

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
        Demo_Update(&s_demo, now, &s_buoy, &s_mass);
#else
        /* --- 3-Second Test Logic (Ignores System Check & Mission) --- */
        /*
        if (!s_sys.done) {
            SystemCheck_Update(&s_sys, now, &hi2c1, &hadc1);
        } else {
            Mission_Update(&s_mission, now, &s_state, &s_buoy);
        }
        */

        static uint32_t s_last_test = 0;
        static bool is_extending = false;

        if ((now - s_last_test) >= 3000) { // Toggle every 3 seconds
            s_last_test = now;
            
            if (is_extending) {
                Act_Retract(&s_buoy, now);
                send_log("[TEST] Current Action: RETRACTING (3 sec)\r\n");
                is_extending = false;
            } else {
                Act_Extend(&s_buoy, now);
                send_log("[TEST] Current Action: EXTENDING (3 sec)\r\n");
                is_extending = true;
            }
        }
#endif
    }

    // Slow tasks @ 1000ms
    if ((now - s_last_slow) >= 1000) {
        s_last_slow = now;

        get_simulated_sensors(&s_state.sensors);

        // Override simulated depth with real MS5837 depth
        if(s_depth_sensor_ok) {
            MS5837_Read(&s_depth_sensor);
            s_state.sensors.depth = s_depth_sensor.depth_m;
            char esp_msg[64];
            snprintf(esp_msg, sizeof(esp_msg), "D:%.2f,T:%.2f\n",
                                 s_depth_sensor.depth_m,
                                 s_depth_sensor.temperature_c);
            HAL_UART_Transmit(&huart3, (uint8_t*)esp_msg, strlen(esp_msg), 50);
        }

        s_state.status = rand() % 3;
        s_state.tinyml_result = rand() % 2;

        App_LogTelemetry();
    }
}
