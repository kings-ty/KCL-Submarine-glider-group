#include "app/app.h"
#include "app/demo.h"
#include "drivers/l298n_stepper.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "drivers/ms5837.h"

// Set to 1 to run the bench-test actuator demo (auto-starts, no serial command needed).
// Set to 0 for normal mission mode (send "CMD:MOTOR_ON" via USB serial to start).
#define DEMO_MODE  0

// Handles created in main.c by CubeMX
extern ADC_HandleTypeDef hadc1;   // PA0 - O2 sensor (ADC1_IN0)
//extern ADC_HandleTypeDef hadc2;   // PA1 - LD20 buoyancy pot (ADC2_IN1)
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;

// ============================================================
// Telemetry buffer for burst transmission to ESP32 upon surfacing
// ============================================================
#define TEL_BUF_SIZE     180   // Max 180 seconds (3 mins) @ 1Hz
#define SURFACE_DEPTH_M   0.5f // Below this depth = near surface
#define SUBMERGE_DEPTH_M  1.0f // Above this depth = judged as submerged

typedef struct {
    uint32_t timestamp_ms;
    float    depth;
    float    temp;
    float    pitch;
    float    roll;
    float    yaw;
    float    voltage;
    float    o2;
    uint8_t  status;
    uint8_t  tinyml;
} TelRecord_t;

static TelRecord_t s_telBuf[TEL_BUF_SIZE];
static uint16_t    s_telHead  = 0;   // Next write position
static uint16_t    s_telCount = 0;   // Number of records stored
static uint8_t     s_wasSubmerged = 0; // Submerged status flag
static float       s_prevDepth    = 0.0f;

extern float yaw, roll, pitch;

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

// ============================================================
// Store one current telemetry record in buffer (called every second)
// ============================================================
static void telBuf_push(float depth, float temp, float voltage, float o2)
{
    TelRecord_t *r = &s_telBuf[s_telHead];
    r->timestamp_ms = HAL_GetTick();
    r->depth   = depth;
    r->temp    = temp;
    r->pitch   = pitch;
    r->roll    = roll;
    r->yaw     = yaw;
    r->voltage = voltage;
    r->o2      = o2;
    r->status  = s_state.status;
    r->tinyml  = s_state.tinyml_result;

    s_telHead = (s_telHead + 1) % TEL_BUF_SIZE;
    if (s_telCount < TEL_BUF_SIZE) s_telCount++;
}

// ============================================================
// Burst transmit entire buffer to ESP32
//   Format: TEL:<ms>,<depth>,<temp>,<pitch>,<roll>,<yaw>,<volt>,<o2>,<st>,<ml>\n
// ============================================================
static void telBuf_sendAll(void)
{
    if (s_telCount == 0) return;

    // Header
    const char *header = "BULK_START\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)header, strlen(header), 100);
    send_log("[TEL] Burst sending to ESP32...\r\n");

    // Transmit in order from oldest record
    uint16_t start = (s_telHead - s_telCount + TEL_BUF_SIZE) % TEL_BUF_SIZE;
    for (uint16_t i = 0; i < s_telCount; i++) {
        uint16_t idx = (start + i) % TEL_BUF_SIZE;
        TelRecord_t *r = &s_telBuf[idx];

        char msg[160];
        int len = snprintf(msg, sizeof(msg),
            "TEL:%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d\n",
            r->timestamp_ms, r->depth, r->temp,
            r->pitch, r->roll, r->yaw,
            r->voltage, r->o2, r->status, r->tinyml);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, len, 200);
        HAL_Delay(5);  // Guard ESP32 UART buffer
    }

    // Footer
    const char *footer = "BULK_END\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)footer, strlen(footer), 100);

    char logmsg[60];
    snprintf(logmsg, sizeof(logmsg), "[TEL] Sent %d records\r\n", s_telCount);
    send_log(logmsg);

    // Clear buffer
    s_telCount = 0;
    s_telHead  = 0;
}

// ============================================================
// App_SetRunMode() - Called when LoRa "RUNNING" command is received
//   run=1: Buoyancy chamber activation (Ascent -> Start cruise)
//   run=0: Stop
// ============================================================
void App_SetRunMode(uint8_t run)
{
    uint32_t now = HAL_GetTick();
    if (run) {
        s_state.is_motor_on = true;
        Act_Extend(&s_buoy, now);   // Ascent -> underwater cruise
        send_log("[APP] RUNNING command: buoy EXTEND\r\n");
    } else {
        s_state.is_motor_on = false;
        Act_Stop(&s_buoy);
        send_log("[APP] STOP command\r\n");
    }
}

// ============================================================
// App_EmergencyAscent() - Maximise buoyancy to surface in emergency
// ============================================================
void App_EmergencyAscent(void)
{
    uint32_t now = HAL_GetTick();
    s_state.is_motor_on = false; // 다른 미션 로직 정지
    Act_Extend(&s_buoy, now);    // Maximize buoyancy (exhaust water -> float)
    send_log("[APP] EMERGENCY ASCENT: buoy EXTEND\r\n");
}

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
    s_buoy.safety_timeout_ms = 15000;
    // LD20 has a potentiometer: Blue wire -> PA1 (ADC2_IN1)
    // Yellow -> 3.3V, White -> GND (not STM32 pins, use power header)
    // In DEMO_MODE run purely timed (no position feedback fighting the demo steps).
    // Set has_pos=1 and hadc in normal mission mode when target_adc is properly set.
    s_buoy.has_pos = 0;
    s_buoy.hadc    = NULL;
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
    // Mission_Init(&s_mission);  // ← 제거: s_mission 미선언, Mission_Init 미존재
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
        Demo_Update(&s_demo, now, &s_buoy, &s_mass, &s_stepper);
#else
        // --- Cruising test (Only works when is_motor_on == true) ---
        if (s_state.is_motor_on) {
            static uint32_t s_last_test = 0;
            static bool is_extending = true;

            if ((now - s_last_test) >= 10000) { // 10s (10000ms) interval
                s_last_test = now;
                
                if (is_extending) {
                    Act_Retract(&s_buoy, now);
                    send_log("[TEST] Target Action: RETRACTING (10 sec)\r\n");
                    is_extending = false;
                } else {
                    Act_Extend(&s_buoy, now);
                    send_log("[TEST] Target Action: EXTENDING (10 sec)\r\n");
                    is_extending = true;
                }
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

        // -- 1. Save to telemetry buffer --
        float cur_depth = s_state.sensors.depth;
        telBuf_push(cur_depth, s_depth_sensor_ok ? s_depth_sensor.temperature_c : 0.0f,
                    s_state.sensors.voltage, s_state.sensors.o2);

        // -- 2. Detect submerge entry --
        if (cur_depth >= SUBMERGE_DEPTH_M) {
            s_wasSubmerged = 1;
        }

        // -- 3. Detect surfacing -> Burst transmit to ESP32 --
        if (s_wasSubmerged && cur_depth < SURFACE_DEPTH_M && s_prevDepth >= SURFACE_DEPTH_M) {
            // Moment when depth goes below 0.5m (Entering surface)
            send_log("[TEL] Surfacing detected! Sending burst to ESP32...\r\n");
            telBuf_sendAll();
            s_wasSubmerged = 0;
        }
        s_prevDepth = cur_depth;

        App_LogTelemetry();
    }
}
