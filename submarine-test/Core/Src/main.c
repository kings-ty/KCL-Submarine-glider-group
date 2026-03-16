/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "app/app.h"
#include "fatfs.h" // For SD Logging (Uncomment after generating FATFS in CubeMX)

extern RTC_HandleTypeDef hrtc; // (Uncomment after generating RTC in CubeMX)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_HANDLE huart2
#define RX_BUFFER_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
GliderState g_glider_state; // Global glider state variable

// --- Non-blocking Delay Variables ---
uint32_t g_last_tx_time = 0;
const uint32_t TX_INTERVAL_MS = 1000; // 1 second (Changed back from 1 minute)

// --- IMU Variables ---
uint8_t bno_data[6];
uint8_t mpu_data[14];
int16_t raw_pitch, raw_roll, raw_yaw;
float pitch, roll, yaw;
uint8_t bno_i2c_addr = 0x28;  // Default BNO055 address
uint8_t mpu_i2c_addr = 0x69;  // MPU9250 address confirmed by user
uint8_t mag_i2c_addr = 0x0C;  // MPU9250 Magnetometer address
bool is_mpu9250 = false;

// UART RX (used if UART interrupt re-enabled)
uint8_t g_rx_data;
uint8_t g_rx_buffer[RX_BUFFER_SIZE];
uint16_t g_rx_index = 0;
uint8_t esp_rx_data;
uint8_t esp_rx_buffer[RX_BUFFER_SIZE];
uint16_t esp_rx_index = 0;

volatile bool g_line_received = false;
char g_process_buffer[RX_BUFFER_SIZE];

// Uncomment the below variables after generating FATFS in CubeMX
// FATFS fs;
// FIL file;
// FRESULT fres;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
void ESP32_Process_Data(const char* data);
void send_log(const char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint32_t adc_raw = 0;
  float oxygen_voltage = 0.0f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  // Initialize glider state
  memset(&g_glider_state, 0, sizeof(GliderState));
  g_glider_state.status = 0; // 0: Normal
  g_glider_state.is_motor_on = false;
  App_Init();
  // Initialize random seed
  srand(HAL_GetTick());

  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart2, &g_rx_data, 1);

  HAL_UART_Receive_IT(&huart3, &esp_rx_data, 1);
  send_log("--- SYSTEM READY (115200 BAUD) ---\r\n");
  // Scan I2C bus for devices
  send_log("[I2C] Scanning bus...\r\n");
  HAL_Delay(100);
  int found = 0;
  for(uint8_t addr = 1; addr < 128; addr++)
  {
      if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
      {
          char msg[50];
          snprintf(msg, sizeof(msg), "[I2C] Device found at 0x%02X\r\n", addr);
          send_log(msg);
          found++;
      }
  }
  if(found == 0)
  {
      send_log("[I2C] No devices found - check wiring!\r\n");
  }

  // Check detected addresses (0x0C, 0x69 might be MPU9250)
  if (found > 0) {
      send_log("[INFO] 0x69/0x0C detected? (Possible MPU9250)\r\n");
  }

  // Initialize BNO055 IMU
  HAL_Delay(100);
  uint8_t bno_id = 0;
  
  // Try address 0x28 first
  if(HAL_I2C_Mem_Read(&hi2c1, (0x28 << 1), 0x00, 1, &bno_id, 1, 100) == HAL_OK && bno_id == 0xA0)
  {
      bno_i2c_addr = 0x28;
      send_log("[IMU] BNO055 detected at 0x28\r\n");
      
      // Set to CONFIG mode
      uint8_t config_mode = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &config_mode, 1, 100);
      HAL_Delay(25);
      
      // Set to NDOF mode (9-axis fusion)
      uint8_t ndof_mode = 0x0C;
      HAL_I2C_Mem_Write(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &ndof_mode, 1, 100);
      HAL_Delay(100);  // Increased delay for sensor stabilization
      
      // Check current mode
      uint8_t current_mode = 0;
      HAL_I2C_Mem_Read(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &current_mode, 1, 100);
      char mode_msg[50];
      snprintf(mode_msg, sizeof(mode_msg), "[IMU] Mode: 0x%02X\r\n", current_mode);
      send_log(mode_msg);
      
      // Check calibration status
      uint8_t calib = 0;
      HAL_I2C_Mem_Read(&hi2c1, (bno_i2c_addr << 1), 0x35, 1, &calib, 1, 100);
      char calib_msg[50];
      snprintf(calib_msg, sizeof(calib_msg), "[IMU] Calib: 0x%02X\r\n", calib);
      send_log(calib_msg);
      
      send_log("[IMU] Initialized OK\r\n");
  }
  // Try address 0x29
  else if(HAL_I2C_Mem_Read(&hi2c1, (0x29 << 1), 0x00, 1, &bno_id, 1, 100) == HAL_OK && bno_id == 0xA0)
  {
      bno_i2c_addr = 0x29;
      send_log("[IMU] BNO055 detected at 0x29\r\n");
      
      uint8_t config_mode = 0x00;
      HAL_I2C_Mem_Write(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &config_mode, 1, 100);
      HAL_Delay(25);
      
      uint8_t ndof_mode = 0x0C;
      HAL_I2C_Mem_Write(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &ndof_mode, 1, 100);
      HAL_Delay(100);  // Increased delay
      
      // Check current mode
      uint8_t current_mode = 0;
      HAL_I2C_Mem_Read(&hi2c1, (bno_i2c_addr << 1), 0x3D, 1, &current_mode, 1, 100);
      char mode_msg[50];
      snprintf(mode_msg, sizeof(mode_msg), "[IMU] Mode: 0x%02X\r\n", current_mode);
      send_log(mode_msg);
      
      send_log("[IMU] Initialized OK at 0x29\r\n");
  }
  else
  {
      send_log("[IMU] BNO055 Not found (0x28/0x29) - Checking for MPU9250...\r\n");
      
      uint8_t mpu_id = 0;
      // Try reading WHO_AM_I (0x75) from 0x69
      HAL_StatusTypeDef mpu_status = HAL_I2C_Mem_Read(&hi2c1, (mpu_i2c_addr << 1), 0x75, 1, &mpu_id, 1, 100);
      
      if(mpu_status == HAL_OK && (mpu_id == 0x71 || mpu_id == 0x70 || mpu_id == 0x11))
      {
          is_mpu9250 = true;
          char m_msg[50];
          snprintf(m_msg, sizeof(m_msg), "[IMU] MPU9250 detected! ID: 0x%02X\r\n", mpu_id);
          send_log(m_msg);
          
          uint8_t data = 0;
          // Reset
          data = 0x80;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x6B, 1, &data, 1, 100);
          HAL_Delay(100);
          
          // Wake up, clock source auto
          data = 0x01;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x6B, 1, &data, 1, 100);
          HAL_Delay(10);
          
          // Config DLPF
          data = 0x03;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x1A, 1, &data, 1, 100);
          
          // Gyro 2000 dps
          data = 0x18;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x1B, 1, &data, 1, 100);
          
          // Accel 4g
          data = 0x08;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x1C, 1, &data, 1, 100);
          
          // Enable I2C bypass for Magnetometer
          data = 0x02;
          HAL_I2C_Mem_Write(&hi2c1, (mpu_i2c_addr << 1), 0x37, 1, &data, 1, 100);
          HAL_Delay(10);
          
          // Init Magnetometer (AK8963)
          data = 0x16; // 16-bit output, 100Hz continuous measurement
          if(HAL_I2C_Mem_Write(&hi2c1, (mag_i2c_addr << 1), 0x0A, 1, &data, 1, 100) == HAL_OK)
          {
              send_log("[IMU] AK8963 Magnetometer initialized\r\n");
          }
          
          send_log("[IMU] MPU9250 Initialized OK\r\n");
      }
      else
      {
          char err_msg[100];
          snprintf(err_msg, sizeof(err_msg), "[IMU] MPU9250 Read Error! Status: %d, ID: 0x%02X\r\n", mpu_status, mpu_id);
          send_log(err_msg);
          send_log("[IMU] No IMU found! (BNO055/MPU9250)\r\n");
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Tick();

    // --- TASK: Handle incoming serial data from PC or ESP32 ---
    if (g_line_received)
    {
        if (strncmp(g_process_buffer, "SDLOG:", 6) == 0)
        {
            // ESP32가 보낸 통합 센서 데이터
            ESP32_Process_Data(g_process_buffer);
        }
        else
        {
            // 기타 시리얼 명령 처리
            process_serial_command((uint8_t*)g_process_buffer, strlen(g_process_buffer));
        }
        g_line_received = false;
    }

    // --- 1. Non-blocking sensor data transmission every second ---
    uint32_t current_time = HAL_GetTick();
    if (current_time - g_last_tx_time >= TX_INTERVAL_MS)
    {
        g_last_tx_time = current_time;

        // --- TASK 1: Periodic Depth Reading & Transmission ---
        // (Mock implementation for MS5837 under I2C1, put real reading function here)
        float ms5837_depth = 1.5f;
        float ms5837_temp = 20.3f;
        
        char depth_msg[50];
        snprintf(depth_msg, sizeof(depth_msg), "D:%.2f,T:%.2f\n", ms5837_depth, ms5837_temp);
        HAL_UART_Transmit(&huart2, (uint8_t*)depth_msg, strlen(depth_msg), 100);

        // 1. Read IMU data
        if (!is_mpu9250)
        {
            // BNO055 Reading
            HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (bno_i2c_addr << 1), 0x1A, 1, bno_data, 6, 100);
            if(status == HAL_OK)
            {
                raw_yaw   = (int16_t)((bno_data[1] << 8) | bno_data[0]);
                raw_roll  = (int16_t)((bno_data[3] << 8) | bno_data[2]);
                raw_pitch = (int16_t)((bno_data[5] << 8) | bno_data[4]);
                
                yaw   = (float)raw_yaw / 16.0f;
                roll  = (float)raw_roll / 16.0f;
                pitch = (float)raw_pitch / 16.0f;
            }
        }
        else
        {
            // MPU9250 Reading (Accel + Gyro)
            if(HAL_I2C_Mem_Read(&hi2c1, (mpu_i2c_addr << 1), 0x3B, 1, mpu_data, 14, 100) == HAL_OK)
            {
                int16_t ax = (int16_t)((mpu_data[0] << 8) | mpu_data[1]);
                int16_t ay = (int16_t)((mpu_data[2] << 8) | mpu_data[3]);
                int16_t az = (int16_t)((mpu_data[4] << 8) | mpu_data[5]);
                // int16_t temp = (int16_t)((mpu_data[6] << 8) | mpu_data[7]);
                // int16_t gx = (int16_t)((mpu_data[8] << 8) | mpu_data[9]);
                // int16_t gy = (int16_t)((mpu_data[10] << 8) | mpu_data[11]);
                // int16_t gz = (int16_t)((mpu_data[12] << 8) | mpu_data[13]);
                
                // Simple Pitch/Roll from Accel
                pitch = atan2f((float)ay, sqrtf((float)ax*ax + (float)az*az)) * 57.29578f;
                roll  = atan2f(-(float)ax, (float)az) * 57.29578f;
                
                // Read Magnetometer for Yaw
                uint8_t mag_status = 0;
                HAL_I2C_Mem_Read(&hi2c1, (mag_i2c_addr << 1), 0x02, 1, &mag_status, 1, 100);
                if(mag_status & 0x01) { // Data Ready
                    uint8_t mag_data[7];
                    if(HAL_I2C_Mem_Read(&hi2c1, (mag_i2c_addr << 1), 0x03, 1, mag_data, 7, 100) == HAL_OK) {
                        int16_t mx = (int16_t)((mag_data[1] << 8) | mag_data[0]);
                        int16_t my = (int16_t)((mag_data[3] << 8) | mag_data[2]);
                        // int16_t mz = (int16_t)((mag_data[5] << 8) | mag_data[4]);
                        yaw = atan2f((float)my, (float)mx) * 57.29578f;
                    }
                }
                
                char dbg[100];
                snprintf(dbg, sizeof(dbg), "[MPU] Acc: %d,%d,%d | P:%.1f R:%.1f Y:%.1f\r\n", ax, ay, az, pitch, roll, yaw);
                send_log(dbg);
            }
        }

        // ADC Oxygen Sensor Reading
        // 1. Start ADC
        HAL_ADC_Start(&hadc1);

        // 2. Wait for conversion complete (timeout 10ms)
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
        {
            // 3. Read digital value (0 ~ 4095)
            adc_raw = HAL_ADC_GetValue(&hadc1);

            // 4. Convert to voltage value (3.3V reference)
            // Formula: V = (ADC_Value * 3.3) / 4095
            oxygen_voltage = (float)adc_raw * (3.3f / 4095.0f);
        }
        // 5. Stop ADC
        HAL_ADC_Stop(&hadc1);

        // 2. Get simulated sensor data
        get_simulated_sensors(&g_glider_state.sensors);
        g_glider_state.status = rand() % 3;
        g_glider_state.tinyml_result = rand() % 2;

        // 3. Format and send all data (Manually format floats for compatibility)
        int v_int = (int)g_glider_state.sensors.voltage;
        int v_dec = (int)((g_glider_state.sensors.voltage - v_int) * 100);
        if (v_dec < 0) v_dec = -v_dec;

        int d_int = (int)g_glider_state.sensors.depth;
        int d_dec = (int)((g_glider_state.sensors.depth - d_int) * 100);
        if (d_dec < 0) d_dec = -d_dec;

        int p_int = (int)pitch;
        int p_dec = (int)((pitch - p_int) * 100);
        if (p_dec < 0) p_dec = -p_dec;

        int r_int = (int)roll;
        int r_dec = (int)((roll - r_int) * 100);
        if (r_dec < 0) r_dec = -r_dec;

        int y_int = (int)yaw;
        int y_dec = (int)((yaw - y_int) * 100);
        if (y_dec < 0) y_dec = -y_dec;

        int o2_v_int = (int)oxygen_voltage;
        int o2_v_dec = (int)((oxygen_voltage - o2_v_int) * 1000); // 3 decimal places
        if (o2_v_dec < 0) o2_v_dec = -o2_v_dec;

        char tx_buffer[256];
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "[TELEMETRY] Y:%d.%02d R:%d.%02d P:%d.%02d | D:%d.%02d V:%d.%02d | O2:%d.%03dV St:%d ML:%d ADC:%lu\r\n",
                 y_int, y_dec, r_int, r_dec, p_int, p_dec,
                 d_int, d_dec, v_int, v_dec,
                 o2_v_int, o2_v_dec,
                 g_glider_state.status,
                 g_glider_state.tinyml_result,
                 adc_raw);
        send_log(tx_buffer);

        // HAL_Delay(1000);
    }

    // --- 2. Other non-blocking tasks can be added here ---
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  // PA0 must be ANALOG for ADC1_IN0 (O2 sensor)
  GPIO_InitTypeDef GPIO_InitStruct_adc = {0};
  GPIO_InitStruct_adc.Pin  = GPIO_PIN_0;
  GPIO_InitStruct_adc.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct_adc.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_adc);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  __HAL_RCC_TIM2_CLK_ENABLE();
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 168-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // PA0 = ADC1_IN0 (O2 sensor), PA1 = ADC2_IN1 (LD20 buoyancy pot)
  // Both must be ANALOG - configured in MX_ADC1_Init / MX_ADC2_Init


  // Configure LED pins for STM32F407VET6 board
  // User LED 1: PA6, User LED 2: PA7 (both are sink mode - LOW to turn on)
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Turn off LEDs initially (HIGH = OFF for sink mode)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  // Motor driver direction pins (DFR0601)
  // Buoyancy actuator: INA=PB0, INB=PB1
  // Mass actuator:     INA=PB10, INB=PB11
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // Start with both drivers stopped (INA=INB=0)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Logs a message via UART.
  * @param  message: The string to send.
  * @retval None
  */
void ESP32_Process_Data(const char* data) {
    // 1. "D:" 글자가 있는 위치 찾기 (데이터 형태: D:1.23,T:20.5...)
    char *d_ptr = strstr(data, "D:");

    if (d_ptr != NULL) {
        // 2. "D:" 바로 뒤에 있는 숫자(1.23)를 실수(float)로 변환
        float depth = atof(d_ptr + 2);

        // 3. 수심 1m 이상이면 전송 허락, 아니면 차단!
        if (depth >= 1.0f) {
            char msg[] = "SEND_OK\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
        } else {
            char msg[] = "SEND_NO\n";
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}

void send_log(const char* message) {
    HAL_UART_Transmit(&UART_HANDLE, (uint8_t*)message, strlen(message), 100);
	CDC_Transmit_FS((uint8_t*)message, strlen(message));
}

/**
  * @brief  Generates simulated sensor data.
  * @param  sensors: Pointer to the GliderSensors struct to fill.
  * @retval None
  */
void get_simulated_sensors(GliderSensors *sensors)
{
    // 1. Voltage simulation (11.0V to 13.0V)
    sensors->voltage = 11.0f + (rand() % 201) / 100.0f;

    // 2. Depth simulation (0.0m to 50.0m, fluctuating up and down)
    static bool depth_increasing = true;
    float depth_change = (rand() % 51) / 100.0f; // 0.0m to 0.5m change

    if (depth_increasing) {
        sensors->depth += depth_change;
        if (sensors->depth >= 50.0f) {
            sensors->depth = 50.0f;
            depth_increasing = false;
        }
    } else {
        sensors->depth -= depth_change;
        if (sensors->depth <= 0.0f) {
            sensors->depth = 0.0f;
            depth_increasing = true;
        }
    }

    // 3. O2 saturation simulation (70.0% to 95.0%)
    sensors->o2 = 70.0f + (rand() % 2501) / 100.0f;
}

/**
  * @brief  Parses and processes a received serial command.
  * @param  buffer: The received data buffer.
  * @param  len: The length of the data.
  * @retval None
  */
void process_serial_command(uint8_t* buffer, uint16_t len)
{
    if (len > 0 && len < RX_BUFFER_SIZE) {
        buffer[len] = '\0'; // Null-terminate the string
    }

    if (strcmp((char*)buffer, "CMD:MOTOR_ON") == 0)
    {
        App_State()->is_motor_on = true;
        send_log("[CMD] Motor Started\r\n");
    }
    else if (strcmp((char*)buffer, "CMD:MOTOR_OFF") == 0)
    {
        App_State()->is_motor_on = false;
        send_log("[CMD] Motor Stopped\r\n");
    }
    else if (strcmp((char*)buffer, "CMD:LED_TOGGLE") == 0)
    {
//        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        send_log("[CMD] LED Toggled\r\n");
    }
    else
    {
        send_log("[CMD] Unknown command\r\n");
    }
}

/**
  * @brief  UART reception complete callback.
  * @param  huart: UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // =======================================================
    // 1. PC 모니터링 및 명령어 수신 (huart2)
    // =======================================================
    if (huart->Instance == huart2.Instance)
    {
        if (g_rx_data == '\n' || g_rx_data == '\r')
        {
            if (g_rx_index > 0)
            {
                g_rx_buffer[g_rx_index] = '\0';
                if (!g_line_received)
                {
                    strncpy(g_process_buffer, (char*)g_rx_buffer, RX_BUFFER_SIZE);
                    g_line_received = true;
                }
                g_rx_index = 0;
                memset(g_rx_buffer, 0, RX_BUFFER_SIZE);
            }
        }
        else
        {
            if (g_rx_index < RX_BUFFER_SIZE - 1) g_rx_buffer[g_rx_index++] = g_rx_data;
        }
        HAL_UART_Receive_IT(&huart2, &g_rx_data, 1);
    }

    // =======================================================
    // 2. ESP32 센서 데이터 수신 (huart3) - 🌟 새로 추가됨 🌟
    // =======================================================
    else if (huart->Instance == huart3.Instance)
    {
        if (esp_rx_data == '\n' || esp_rx_data == '\r')
        {
            if (esp_rx_index > 0)
            {
                esp_rx_buffer[esp_rx_index] = '\0';

                // ESP32에서 데이터가 오면, 똑같이 메인 루프(g_process_buffer)로 넘겨줍니다!
                if (!g_line_received)
                {
                    strncpy(g_process_buffer, (char*)esp_rx_buffer, RX_BUFFER_SIZE);
                    g_line_received = true; // 플래그 ON!
                }
                esp_rx_index = 0;
                memset(esp_rx_buffer, 0, RX_BUFFER_SIZE);
            }
        }
        else
        {
            if (esp_rx_index < RX_BUFFER_SIZE - 1) esp_rx_buffer[esp_rx_index++] = esp_rx_data;
        }
        HAL_UART_Receive_IT(&huart3, &esp_rx_data, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
