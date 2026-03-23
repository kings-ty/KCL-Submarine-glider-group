/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ==================== 数据类型定义 ====================
typedef struct {
int16_t x;
int16_t y;
int16_t z;
} imu_vector_t;

typedef struct {
imu_vector_t accel;
imu_vector_t gyro;
imu_vector_t mag;
int16_t temperature;
} imu_9dof_data_t;

// ==================== 姿态数据结构 ====================
typedef struct {
float roll;
float pitch;
float heading;
} attitude_t;

// ==================== 电机动作状态枚举 ====================
typedef enum {
ACTION_STOP,
ACTION_FORWARD,
ACTION_BACKWARD
} MotorAction_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ==================== ICM20600 定义 ====================
#define ICM20600_WHO_AM_I 0x75
#define ICM20600_WHO_AM_I_VALUE 0x11
#define ICM20600_PWR_MGMT_1 0x6B
#define ICM20600_PWR_MGMT_2 0x6C
#define ICM20600_ACCEL_CONFIG 0x1C
#define ICM20600_GYRO_CONFIG 0x1B
#define ICM20600_CONFIG 0x1A
#define ICM20600_ACCEL_XOUT_H 0x3B
#define ICM20600_GYRO_XOUT_H 0x43
#define ICM20600_TEMP_OUT_H 0x41

// ==================== AK09918 定义 ====================
#define AK09918_ADDRESS (0x0C << 1)
#define AK09918_WIA1 0x00
#define AK09918_WIA2 0x01
#define AK09918_COMPANY_ID 0x48
#define AK09918_DEVICE_ID 0x0C
#define AK09918_HXL 0x11
#define AK09918_CNTL2 0x31
#define AK09918_CNTL1 0x30
#define AK09918_ST1 0x10
#define AK09918_ST2 0x18
#define AK09918_MODE_POWER_DOWN 0x00
#define AK09918_MODE_CONTINUOUS_100HZ 0x08

// ==================== 电机控制参数 ====================
#define TARGET_ROLL 0.0f // 目标横滚角 (度)
#define TARGET_PITCH 5.0f // 目标俯仰角 (度)
#define ANGLE_DEADBAND 1.5f // 死区大小 (±1.5度)
#define ANGLE_HYSTERESIS 0.5f // 滞回值
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ==================== 全局变量 ====================
uint8_t imu_initialized = 0;
uint8_t icm20600_addr = 0x69;
uint32_t i2c_error_count = 0;

imu_9dof_data_t imu_data;
attitude_t attitude;

// 电机状态变量
static MotorAction_t roll_action = ACTION_STOP;
static MotorAction_t pitch_action = ACTION_STOP;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// ==================== IMU函数声明 ====================
void I2C_Init(void);
void I2C_RecoverBus(void);
uint8_t ICM20600_Check(void);
uint8_t ICM20600_Init(void);
HAL_StatusTypeDef ICM20600_ReadAccel(imu_vector_t *accel);
HAL_StatusTypeDef ICM20600_ReadGyro(imu_vector_t *gyro);
HAL_StatusTypeDef ICM20600_ReadTemp(int16_t *temp);
uint8_t AK09918_Check(void);
uint8_t AK09918_Init(void);
uint8_t AK09918_ReadMag(imu_vector_t *mag);
uint8_t IMU_Init(void);
uint8_t IMU_ReadAll(imu_9dof_data_t *data);
void CalculateAttitude(imu_9dof_data_t *data, attitude_t *att);
void PrintAttitude(attitude_t *att);
void I2C_Scan(void);

// ==================== 电机控制函数声明 ====================
static void Control_Roll(float current_roll);
static void Control_Pitch(float current_pitch);
static void Set_Roll_Motor(MotorAction_t action);
static void Set_Pitch_Motor(MotorAction_t action);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// ==================== I2C读写函数 ====================
uint8_t I2C_ReadReg(uint16_t dev_addr, uint8_t reg, uint8_t *data) {
return HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

uint8_t I2C_WriteReg(uint16_t dev_addr, uint8_t reg, uint8_t data) {
return HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

uint8_t I2C_ReadRegs(uint16_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) {
return HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

// ==================== 重定向printf ====================
int fputc(int ch, FILE *f) {
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
return ch;
}

// ==================== I2C总线恢复函数 ====================
void I2C_RecoverBus(void) {
GPIO_InitTypeDef GPIO_InitStruct = {0};

printf("\r\n⚠️ I2C bus error detected! Recovering...\r\n");

HAL_I2C_DeInit(&hi2c1);

GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

for(int i = 0; i < 9; i++) {
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
HAL_Delay(1);
}

HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
HAL_Delay(1);

HAL_Delay(10);
I2C_Init();

printf("I2C bus recovered!\r\n");
i2c_error_count = 0;
}

// ==================== I2C初始化 ====================
void I2C_Init(void) {
__HAL_RCC_I2C1_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();

GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
GPIO_InitStruct.Pull = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

hi2c1.Instance = I2C1;
hi2c1.Init.ClockSpeed = 100000;
hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
hi2c1.Init.OwnAddress1 = 0;
hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
hi2c1.Init.OwnAddress2 = 0;
hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
HAL_I2C_Init(&hi2c1);
}

// ==================== I2C扫描函数 ====================
void I2C_Scan(void) {
uint8_t found_devices = 0;
char buf[32];

HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nScanning I2C bus...\r\n", 25, 100);

for (uint8_t addr = 1; addr < 127; addr++) {
if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 3, 50) == HAL_OK) {
sprintf(buf, "Found device at 0x%02X\r\n", addr);
HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
found_devices++;
}
}

if (found_devices == 0) {
HAL_UART_Transmit(&huart2, (uint8_t*)"No I2C devices found!\r\n", 24, 100);
} else {
sprintf(buf, "Total %d devices found\r\n", found_devices);
HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
}
}

// ==================== ICM20600 函数 ====================
uint8_t ICM20600_Check(void) {
uint8_t who_am_i = 0;
char buf[64];

if (HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), ICM20600_WHO_AM_I,
I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) == HAL_OK) {
if (who_am_i == ICM20600_WHO_AM_I_VALUE) {
icm20600_addr = 0x68;
sprintf(buf, "ICM20600 found at 0x68 (ID=0x%02X)\r\n", who_am_i);
HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
return 1;
}
}

if (HAL_I2C_Mem_Read(&hi2c1, (0x69 << 1), ICM20600_WHO_AM_I,
I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 100) == HAL_OK) {
if (who_am_i == ICM20600_WHO_AM_I_VALUE) {
icm20600_addr = 0x69;
sprintf(buf, "ICM20600 found at 0x69 (ID=0x%02X)\r\n", who_am_i);
HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
return 1;
}
}
return 0;
}

uint8_t ICM20600_Init(void) {
uint16_t dev_addr = icm20600_addr << 1;

printf(" Resetting...");
if (I2C_WriteReg(dev_addr, ICM20600_PWR_MGMT_1, 0x80) != HAL_OK) {
printf(" FAILED\r\n");
return 0;
}
printf(" OK\r\n");
HAL_Delay(100);

printf(" Waking up...");
if (I2C_WriteReg(dev_addr, ICM20600_PWR_MGMT_1, 0x01) != HAL_OK) {
printf(" FAILED\r\n");
return 0;
}
printf(" OK\r\n");
HAL_Delay(10);

printf(" Configuring gyro...");
I2C_WriteReg(dev_addr, ICM20600_GYRO_CONFIG, 0x18);
printf(" OK\r\n");

printf(" Configuring accel...");
I2C_WriteReg(dev_addr, ICM20600_ACCEL_CONFIG, 0x18);
printf(" OK\r\n");

printf(" Configuring filter...");
I2C_WriteReg(dev_addr, ICM20600_CONFIG, 0x01);
printf(" OK\r\n");

return 1;
}

HAL_StatusTypeDef ICM20600_ReadAccel(imu_vector_t *accel) {
uint16_t dev_addr = icm20600_addr << 1;
uint8_t data[6];
HAL_StatusTypeDef status = I2C_ReadRegs(dev_addr, ICM20600_ACCEL_XOUT_H, data, 6);

if (status == HAL_OK) {
accel->x = (int16_t)(data[0] << 8 | data[1]);
accel->y = (int16_t)(data[2] << 8 | data[3]);
accel->z = (int16_t)(data[4] << 8 | data[5]);
}
return status;
}

HAL_StatusTypeDef ICM20600_ReadGyro(imu_vector_t *gyro) {
uint16_t dev_addr = icm20600_addr << 1;
uint8_t data[6];
HAL_StatusTypeDef status = I2C_ReadRegs(dev_addr, ICM20600_GYRO_XOUT_H, data, 6);

if (status == HAL_OK) {
gyro->x = (int16_t)(data[0] << 8 | data[1]);
gyro->y = (int16_t)(data[2] << 8 | data[3]);
gyro->z = (int16_t)(data[4] << 8 | data[5]);
}
return status;
}

HAL_StatusTypeDef ICM20600_ReadTemp(int16_t *temp) {
uint16_t dev_addr = icm20600_addr << 1;
uint8_t data[2];
HAL_StatusTypeDef status = I2C_ReadRegs(dev_addr, ICM20600_TEMP_OUT_H, data, 2);

if (status == HAL_OK) {
*temp = (int16_t)(data[0] << 8 | data[1]);
}
return status;
}

// ==================== AK09918 函数 ====================
uint8_t AK09918_Check(void) {
uint8_t company_id = 0;
uint8_t device_id = 0;
char buf[64];

if (HAL_I2C_Mem_Read(&hi2c1, AK09918_ADDRESS, AK09918_WIA1,
I2C_MEMADD_SIZE_8BIT, &company_id, 1, 100) == HAL_OK) {
HAL_I2C_Mem_Read(&hi2c1, AK09918_ADDRESS, AK09918_WIA2,
I2C_MEMADD_SIZE_8BIT, &device_id, 1, 100);

sprintf(buf, "AK09918 Company ID=0x%02X, Device ID=0x%02X\r\n", company_id, device_id);
HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);

return (company_id == AK09918_COMPANY_ID && device_id == AK09918_DEVICE_ID);
}
return 0;
}

uint8_t AK09918_Init(void) {
uint8_t status;
uint8_t chip_id;
uint8_t cntl1_val;
uint8_t cntl2_val;
uint8_t test_val;

printf("\r\n=== AK09918 Debug ===\r\n");

I2C_ReadReg(AK09918_ADDRESS, AK09918_WIA2, &chip_id);
printf("WIA2 = 0x%02X (should be 0x%02X)\r\n", chip_id, AK09918_DEVICE_ID);

if (chip_id != AK09918_DEVICE_ID) {
printf("❌ AK09918 ID mismatch!\r\n");
return 0;
}

I2C_ReadReg(AK09918_ADDRESS, 0x31, &cntl2_val);
I2C_ReadReg(AK09918_ADDRESS, 0x30, &cntl1_val);
printf("Before init - CNTL1=0x%02X, CNTL2=0x%02X\r\n", cntl1_val, cntl2_val);

I2C_WriteReg(AK09918_ADDRESS, 0x32, 0x01);
HAL_Delay(10);

I2C_ReadReg(AK09918_ADDRESS, 0x31, &cntl2_val);
I2C_ReadReg(AK09918_ADDRESS, 0x30, &cntl1_val);
printf("After reset - CNTL1=0x%02X, CNTL2=0x%02X\r\n", cntl1_val, cntl2_val);

printf("Setting power down mode...");
I2C_WriteReg(AK09918_ADDRESS, AK09918_CNTL2, AK09918_MODE_POWER_DOWN);
HAL_Delay(10);
I2C_ReadReg(AK09918_ADDRESS, 0x31, &cntl2_val);
printf(" CNTL2=0x%02X\r\n", cntl2_val);

printf("Setting continuous mode 100Hz...");
if (I2C_WriteReg(AK09918_ADDRESS, AK09918_CNTL2, AK09918_MODE_CONTINUOUS_100HZ) != HAL_OK) {
printf(" FAILED\r\n");
return 0;
}
HAL_Delay(10);

I2C_ReadReg(AK09918_ADDRESS, AK09918_CNTL2, &status);
printf(" CNTL2=0x%02X ", status);

I2C_ReadReg(AK09918_ADDRESS, AK09918_CNTL1, &cntl1_val);
printf("CNTL1=0x%02X\r\n", cntl1_val);

I2C_WriteReg(AK09918_ADDRESS, 0x30, 0x5A);
HAL_Delay(1);
I2C_ReadReg(AK09918_ADDRESS, 0x30, &test_val);
printf("Write test - wrote 0x5A, read 0x%02X\r\n", test_val);

I2C_WriteReg(AK09918_ADDRESS, 0x30, cntl1_val);

printf("=== End Debug ===\r\n");

return (status == AK09918_MODE_CONTINUOUS_100HZ);
}

uint8_t AK09918_ReadMag(imu_vector_t *mag) {
uint8_t data[6];
uint8_t st1, st2;

I2C_ReadReg(AK09918_ADDRESS, AK09918_ST1, &st1);

if (!(st1 & 0x01)) {
static uint32_t no_data_cnt = 0;
if (no_data_cnt++ % 50 == 0) {
printf("No mag data (ST1=0x%02X)\r\n", st1);
}
return 0;
}

if (I2C_ReadRegs(AK09918_ADDRESS, AK09918_HXL, data, 6) == HAL_OK) {
mag->x = (int16_t)(data[1] << 8 | data[0]);
mag->y = (int16_t)(data[3] << 8 | data[2]);
mag->z = (int16_t)(data[5] << 8 | data[4]);

I2C_ReadReg(AK09918_ADDRESS, AK09918_ST2, &st2);

static uint32_t print_cnt = 0;
if (print_cnt++ % 10 == 0 || mag->x != 0 || mag->y != 0) {
printf("\r\nAK09918: ST1=0x%02X, ST2=0x%02X, X=%6d Y=%6d Z=%6d ",
st1, st2, mag->x, mag->y, mag->z);
}

return 1;
}
return 0;
}

// ==================== IMU综合函数 ====================
uint8_t IMU_Init(void) {
HAL_UART_Transmit(&huart2, (uint8_t*)"\r\nChecking ICM20600...\r\n", 25, 100);

if (!ICM20600_Check()) {
HAL_UART_Transmit(&huart2, (uint8_t*)"❌ ICM20600 not detected!\r\n", 28, 100);
return 0;
}

HAL_UART_Transmit(&huart2, (uint8_t*)"Checking AK09918...\r\n", 22, 100);
if (!AK09918_Check()) {
HAL_UART_Transmit(&huart2, (uint8_t*)"❌ AK09918 not detected!\r\n", 27, 100);
return 0;
}

HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing ICM20600...\r\n", 26, 100);
if (!ICM20600_Init()) {
HAL_UART_Transmit(&huart2, (uint8_t*)" FAILED\r\n", 10, 100);
return 0;
}
HAL_UART_Transmit(&huart2, (uint8_t*)" OK\r\n", 6, 100);

HAL_UART_Transmit(&huart2, (uint8_t*)"Initializing AK09918...\r\n", 25, 100);
if (!AK09918_Init()) {
HAL_UART_Transmit(&huart2, (uint8_t*)" FAILED\r\n", 10, 100);
return 0;
}
HAL_UART_Transmit(&huart2, (uint8_t*)" OK\r\n", 6, 100);

return 1;
}

uint8_t IMU_ReadAll(imu_9dof_data_t *data) {
uint8_t success = 1;

if (ICM20600_ReadAccel(&data->accel) != HAL_OK) success = 0;
if (ICM20600_ReadGyro(&data->gyro) != HAL_OK) success = 0;
if (ICM20600_ReadTemp(&data->temperature) != HAL_OK) success = 0;

AK09918_ReadMag(&data->mag);

return success;
}

// ==================== 姿态计算函数 ====================
void CalculateAttitude(imu_9dof_data_t *data, attitude_t *att) {
float ax = data->accel.x * 16.0f / 32768.0f;
float ay = data->accel.y * 16.0f / 32768.0f;
float az = data->accel.z * 16.0f / 32768.0f;

float mx = data->mag.x * 0.15f;
float my = data->mag.y * 0.15f;
float mz = data->mag.z * 0.15f;

att->roll = atan2f(ay, az);
float accel_norm = sqrtf(ay*ay + az*az);
att->pitch = atan2f(-ax, accel_norm);

float cos_roll = cosf(att->roll);
float sin_roll = sinf(att->roll);
float cos_pitch = cosf(att->pitch);
float sin_pitch = sinf(att->pitch);

float Xh = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
float Yh = my * cos_roll - mz * sin_roll;

float heading_rad = atan2f(Yh, Xh);
att->heading = heading_rad * 180.0f / 3.14159f;

att->heading += -2.0f;

if (att->heading < 0) att->heading += 360.0f;
if (att->heading >= 360.0f) att->heading -= 360.0f;

att->roll *= 180.0f / 3.14159f;
att->pitch *= 180.0f / 3.14159f;
}

// ==================== 打印姿态函数 ====================
void PrintAttitude(attitude_t *att) {
char buffer[64];
sprintf(buffer, "\r\nRoll: %6.2f° Pitch: %6.2f° Heading: %6.2f° ",
att->roll, att->pitch, att->heading);
HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}
/* USER CODE BEGIN 4 */
// ==================== 新增：打印PA4-PA8引脚状态 ====================
void Print_GPIO_Status(void) {
char buffer[64];

// 读取GPIOA端口的输入数据寄存器
uint16_t gpio_idr = GPIOA->IDR;

// 解析每个引脚的状态 (0 或 1)
uint8_t pa4_state = (gpio_idr & GPIO_PIN_4) ? 1 : 0;
uint8_t pa5_state = (gpio_idr & GPIO_PIN_5) ? 1 : 0;
uint8_t pa6_state = (gpio_idr & GPIO_PIN_6) ? 1 : 0;
uint8_t pa7_state = (gpio_idr & GPIO_PIN_7) ? 1 : 0;
uint8_t pa8_state = (gpio_idr & GPIO_PIN_8) ? 1 : 0;

// 检查PWM是否正在输出 (对于PA7和PA8，可以通过定时器状态判断)
uint8_t tim1_running = (htim1.State == HAL_TIM_STATE_BUSY) ? 1 : 0;
uint8_t tim3_running = (htim3.State == HAL_TIM_STATE_BUSY) ? 1 : 0;

sprintf(buffer, "GPIO: PA4=%d PA5=%d PA6=%d | PWM: PA7(TIM3)=%d PA8(TIM1)=%d | TIM1:%d TIM3:%d\r\n",
pa4_state, pa5_state, pa6_state,
pa7_state, pa8_state,
tim1_running, tim3_running);

HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
}
// ==================== 电机控制函数 ====================
static void Set_Roll_Motor(MotorAction_t action) {
switch (action) {
case ACTION_STOP:
// 停止步进电机 - 停止发送脉冲
HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
break;
case ACTION_FORWARD:
// 设置方向为正转 (假设PA4高电平为正转)
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
// 启动PWM输出脉冲
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000); // 50%占空比
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
break;
case ACTION_BACKWARD:
// 设置方向为反转 (假设PA4低电平为反转)
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
// 启动PWM输出脉冲
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
break;
}
}

static void Set_Pitch_Motor(MotorAction_t action) {
switch (action) {
case ACTION_STOP:
// 停止推杆 - PWM设为0
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
break;
case ACTION_FORWARD:
// 推杆伸出 (假设PA6高电平为伸出)
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
// 设置PWM为合适速度 (500 = 50%占空比)
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
break;
case ACTION_BACKWARD:
// 推杆缩回 (假设PA6低电平为缩回)
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
// 设置PWM为合适速度
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
break;
}
}

static void Control_Roll(float current_roll) {
float error = TARGET_ROLL - current_roll;

switch (roll_action) {
case ACTION_STOP:
if (error > (ANGLE_DEADBAND + ANGLE_HYSTERESIS)) {
roll_action = ACTION_FORWARD;
Set_Roll_Motor(ACTION_FORWARD);
printf("Roll Start Forward\r\n");
} else if (error < -(ANGLE_DEADBAND + ANGLE_HYSTERESIS)) {
roll_action = ACTION_BACKWARD;
Set_Roll_Motor(ACTION_BACKWARD);
printf("Roll Start Backward\r\n");
}
break;

case ACTION_FORWARD:
if (error < ANGLE_DEADBAND) {
roll_action = ACTION_STOP;
Set_Roll_Motor(ACTION_STOP);
printf("Roll Stop\r\n");
}
break;

case ACTION_BACKWARD:
if (error > -ANGLE_DEADBAND) {
roll_action = ACTION_STOP;
Set_Roll_Motor(ACTION_STOP);
printf("Roll Stop\r\n");
}
break;
}
}

static void Control_Pitch(float current_pitch) {
float error = TARGET_PITCH - current_pitch;

switch (pitch_action) {
case ACTION_STOP:
if (error > (ANGLE_DEADBAND + ANGLE_HYSTERESIS)) {
pitch_action = ACTION_FORWARD;
Set_Pitch_Motor(ACTION_FORWARD);
printf("Pitch Start Forward\r\n");
} else if (error < -(ANGLE_DEADBAND + ANGLE_HYSTERESIS)) {
pitch_action = ACTION_BACKWARD;
Set_Pitch_Motor(ACTION_BACKWARD);
printf("Pitch Start Backward\r\n");
}
break;

case ACTION_FORWARD:
if (error < ANGLE_DEADBAND) {
pitch_action = ACTION_STOP;
Set_Pitch_Motor(ACTION_STOP);
printf("Pitch Stop\r\n");
}
break;

case ACTION_BACKWARD:
if (error > -ANGLE_DEADBAND) {
pitch_action = ACTION_STOP;
Set_Pitch_Motor(ACTION_STOP);
printf("Pitch Stop\r\n");
}
break;
}
}
/* USER CODE END 0 */

/**
* @brief The application entry point.
* @retval int
*/
int main(void)
{
/* USER CODE BEGIN 1 */

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
MX_I2C1_Init();
MX_USART2_UART_Init();
MX_TIM1_Init();
MX_TIM3_Init();
/* USER CODE BEGIN 2 */
// 初始化I2C
printf("\r\n========================================\r\n");
printf("STM32F407 + ICM20600 + AK09918 Test\r\n");
printf("========================================\r\n\r\n");

printf("Initializing I2C...\r\n");
I2C_Init();

I2C_Scan();

imu_initialized = IMU_Init();

if (imu_initialized) {
printf("\r\n✅ IMU initialized successfully!\r\n");
printf("========================================\r\n\r\n");
} else {
printf("\r\n❌ IMU initialization failed!\r\n");
}

// 初始化电机控制引脚为停止状态
Set_Roll_Motor(ACTION_STOP);
Set_Pitch_Motor(ACTION_STOP);
/* USER CODE END 2 */

/* Infinite loop */
/* USER CODE BEGIN WHILE */
uint32_t count = 0;
while (1)
{
if (imu_initialized) {
IMU_ReadAll(&imu_data);
CalculateAttitude(&imu_data, &attitude);

// 每5次循环打印姿态和GPIO状态
if (count % 5 == 0) {
PrintAttitude(&attitude); // 打印姿态角
Print_GPIO_Status(); // 打印GPIO状态
}

// 每5次循环执行一次电机控制
if (count % 5 == 0) {
Control_Roll(attitude.roll);
Control_Pitch(attitude.pitch);
}

count++;
HAL_Delay(100);
} else {
printf(".");
fflush(stdout);
HAL_Delay(1000);
}
}
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
}
/* USER CODE END 3 */

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
RCC_OscInitStruct.PLL.PLLM = 8;
RCC_OscInitStruct.PLL.PLLN = 336;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 4;
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
hi2c1.Init.ClockSpeed = 400000;
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
* @brief TIM1 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM1_Init(void)
{

/* USER CODE BEGIN TIM1_Init 0 */

/* USER CODE END TIM1_Init 0 */

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

/* USER CODE BEGIN TIM1_Init 1 */

/* USER CODE END TIM1_Init 1 */
htim1.Instance = TIM1;
htim1.Init.Prescaler = 84-1;
htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
htim1.Init.Period = 2000-1;
htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim1.Init.RepetitionCounter = 0;
htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
{
Error_Handler();
}
sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
{
Error_Handler();
}
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
sConfigOC.Pulse = 1000;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
* @brief TIM3 Initialization Function
* @param None
* @retval None
*/
static void MX_TIM3_Init(void)
{

/* USER CODE BEGIN TIM3_Init 0 */

/* USER CODE END TIM3_Init 0 */

TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN TIM3_Init 1 */

/* USER CODE END TIM3_Init 1 */
htim3.Instance = TIM3;
htim3.Init.Prescaler = 168-1;
htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
htim3.Init.Period = 1000-1;
htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
{
Error_Handler();
}
sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
{
Error_Handler();
}
sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
{
Error_Handler();
}
/* USER CODE BEGIN TIM3_Init 2 */

/* USER CODE END TIM3_Init 2 */
HAL_TIM_MspPostInit(&htim3);

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

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

/*Configure GPIO pins : PA4 PA5 PA6 */
GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
* @brief This function is executed in case of error occurrence.
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
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
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
