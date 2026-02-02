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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_HANDLE huart2
#define RX_BUFFER_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
GliderState g_glider_state; // Global glider state variable

// --- Non-blocking Delay Variables ---
uint32_t g_last_tx_time = 0;
const uint32_t TX_INTERVAL_MS = 1000; // 1 second

// --- IMU Variables ---
uint8_t bno_data[6];
int16_t raw_pitch, raw_roll, raw_yaw;
float pitch, roll, yaw;
uint8_t bno_i2c_addr = 0x28;  // Default address, will be updated if found

// --- UART RX Handling Variables ---
uint8_t g_rx_data;                        // 1-byte receive data
uint8_t g_rx_buffer[RX_BUFFER_SIZE];      // Receive buffer
uint16_t g_rx_index = 0;                  // Receive buffer index
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  uint32_t adc_raw;      // ADC raw value (0~4095)
  float oxygen_voltage;  // Voltage converted value (V)

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
  /* USER CODE BEGIN 2 */
  // Initialize glider state
  memset(&g_glider_state, 0, sizeof(GliderState));
  g_glider_state.status = 0; // 0: Normal
  g_glider_state.is_motor_on = false;

  // Initialize random seed
  srand(HAL_GetTick());

  // Start UART reception in interrupt mode
  // HAL_UART_Receive_IT(&UART_HANDLE, &g_rx_data, 1);  // Temporarily disabled

  send_log("--- TEST START 9600 BAUD ---\r\n");

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
      send_log("[IMU] Not found - check I2C connection\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // --- 1. Non-blocking sensor data transmission every second ---
    uint32_t current_time = HAL_GetTick();
    if (current_time - g_last_tx_time >= TX_INTERVAL_MS)
    {
        g_last_tx_time = current_time;

        // 1. Read IMU data (use detected address)
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, (bno_i2c_addr << 1), 0x1A, 1, bno_data, 6, 100);
        if(status == HAL_OK)
        {
            raw_yaw = (int16_t)((bno_data[1] << 8) | bno_data[0]);
            raw_roll = (int16_t)((bno_data[3] << 8) | bno_data[2]);
            raw_pitch = (int16_t)((bno_data[5] << 8) | bno_data[4]);
            
            yaw = (float)raw_yaw / 16.0f;
            roll = (float)raw_roll / 16.0f;
            pitch = (float)raw_pitch / 16.0f;
            
            // Debug: show raw data every time (no limit)
            char dbg[100];
            snprintf(dbg, sizeof(dbg), "[DBG] Raw: %d,%d,%d\r\n", raw_pitch, raw_roll, raw_yaw);
            send_log(dbg);
        }
        else
        {
            // Debug: I2C read failed
            static int err_count = 0;
            if(err_count < 3)
            {
                send_log("[IMU] Read error\r\n");
                err_count++;
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

        // 3. Format and send all data
        char tx_buffer[256];
        int len = snprintf(tx_buffer, sizeof(tx_buffer),
                 "V:%.2f, D:%.2f, O2_V:%.4f, St:%d, ML:%d, P:%.2f, R:%.2f, Y:%.2f, O2_Raw:%lu\r\n",
                 g_glider_state.sensors.voltage,
                 g_glider_state.sensors.depth,
                 oxygen_voltage,
                 g_glider_state.status,
                 g_glider_state.tinyml_result,
                 pitch, roll, yaw,
                 adc_raw);
        send_log(tx_buffer);

        HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  
  // Configure PA0 as Trigger (Output) and PA1 as Echo (Input)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  // Configure LED pins for STM32F407VET6 board
  // User LED 1: PA6, User LED 2: PA7 (both are sink mode - LOW to turn on)
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Turn off LEDs initially (HIGH = OFF for sink mode)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Logs a message via UART.
  * @param  message: The string to send.
  * @retval None
  */
void send_log(const char* message) {
    // HAL_UART_Transmit(&UART_HANDLE, (uint8_t*)message, strlen(message), 100);
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
        g_glider_state.is_motor_on = true;
        send_log("[CMD] Motor Started\r\n");
    }
    else if (strcmp((char*)buffer, "CMD:MOTOR_OFF") == 0)
    {
        g_glider_state.is_motor_on = false;
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
    if (huart->Instance == UART_HANDLE.Instance)
    {
        // On newline character, process the command
        if (g_rx_data == '\n' || g_rx_data == '\r')
        {
            if (g_rx_index > 0)
            {
                process_serial_command(g_rx_buffer, g_rx_index);
                g_rx_index = 0;
                memset(g_rx_buffer, 0, RX_BUFFER_SIZE);
            }
        }
        else // Otherwise, append to buffer
        {
            if (g_rx_index < RX_BUFFER_SIZE - 1)
            {
                g_rx_buffer[g_rx_index++] = g_rx_data;
            }
        }
        // Re-arm UART reception interrupt
        HAL_UART_Receive_IT(&UART_HANDLE, &g_rx_data, 1);
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
