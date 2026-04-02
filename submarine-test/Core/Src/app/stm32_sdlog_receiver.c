/*
 * stm32_sdlog_receiver.c - Adjusted version for integration into main.c
 *
 * ⚠️ This file does not include HAL_UART_RxCpltCallback.
 *    It is already in main.c, so call sdlog_parse_and_save() / sdlog_check_emergency_byte() from there.
 */

#include "stm32_sdlog_receiver.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

// ============================================================
// External UART handle (declared in main.c)
// ============================================================
extern UART_HandleTypeDef huart3;

// ============================================================
// Emergency ascent flag (used as extern in main.c)
// ============================================================
volatile uint8_t g_emergencyAscent = 0;
volatile uint8_t g_emergencyReason = 0;

// ============================================================
// FatFS internal variables
// ============================================================
static FATFS  fatfs;
static FIL    logFile;
static uint8_t sdMounted  = 0;
static uint8_t fileOpened = 0;
static uint32_t lineCount = 0;

// ============================================================
// EmergencyCommand binary reception state machine (byte units)
// ============================================================
#pragma pack(push, 1)
typedef struct {
    uint16_t header;
    uint8_t  command;
    uint8_t  reason;
    uint8_t  checksum;
} EmergencyCommand_t;
#pragma pack(pop)

static uint8_t emgBuf[sizeof(EmergencyCommand_t)];
static uint8_t emgLen = 0;
static uint8_t emgActive = 0;

// ============================================================
// SD card mount & CSV header write (Internal function)
// ============================================================
static HAL_StatusTypeDef sdlog_mount(void) {
    FRESULT res = f_mount(&fatfs, "", 1);
    if (res != FR_OK) return HAL_ERROR;
    sdMounted = 1;

    res = f_open(&logFile, "sdlog.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
    if (res != FR_OK) return HAL_ERROR;

    f_lseek(&logFile, f_size(&logFile));

    if (f_size(&logFile) == 0) {
        const char* header =
            "Timestamp_ms,Depth_m,Temp_C,pH,EC_mS,DO_mg,O2_mg,ML_State,Anomaly_Count\r\n";
        UINT bw;
        f_write(&logFile, header, strlen(header), &bw);
        f_sync(&logFile);
    }

    fileOpened = 1;
    return HAL_OK;
}

// ============================================================
// 📌 Public function 1: sdlog_init()
//    Call immediately after MX_FATFS_Init() in main.c.
// ============================================================
HAL_StatusTypeDef sdlog_init(void) {
    if (sdlog_mount() != HAL_OK) {
        // Program continues even if SD mount fails
        return HAL_ERROR;
    }
    return HAL_OK;
}

// ============================================================
// 📌 Public function 2: sdlog_parse_and_save()
//    Call inside ESP32_Process_Data().
//    Parse format: "SDLOG:D:...,T:...,PH:...,EC:...,aDO:...,O2:...,ML:...,ANM:..."
// ============================================================
void sdlog_parse_and_save(const char* line) {
    if (strncmp(line, "SDLOG:", 6) != 0) return;  // Ignore if not SDLOG:

    const char* p = line + 6;
    float depth = 0, temp = 0, ph = 0, ec = 0, ado = 0, o2 = 0;
    int   mlState = 0, anomCount = 0;

    char* ptr;
    ptr = strstr(p, "D:");   if (ptr) depth     = atof(ptr + 2);
    ptr = strstr(p, "T:");   if (ptr) temp      = atof(ptr + 2);
    ptr = strstr(p, "PH:");  if (ptr) ph        = atof(ptr + 3);
    ptr = strstr(p, "EC:");  if (ptr) ec        = atof(ptr + 3);
    ptr = strstr(p, "aDO:"); if (ptr) ado       = atof(ptr + 4);
    ptr = strstr(p, "O2:");  if (ptr) o2        = atof(ptr + 3);
    ptr = strstr(p, "ML:");  if (ptr) mlState   = atoi(ptr + 3);
    ptr = strstr(p, "ANM:"); if (ptr) anomCount = atoi(ptr + 4);

    if (!sdMounted || !fileOpened) return;

    char csvLine[220];
    int len = snprintf(csvLine, sizeof(csvLine),
        "%lu,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d\r\n",
        HAL_GetTick(), depth, temp, ph, ec, ado, o2, mlState, anomCount);

    UINT bw;
    f_write(&logFile, csvLine, len, &bw);
    lineCount++;

    if (lineCount % 10 == 0) f_sync(&logFile);  // Flush every 10 lines
}

// ============================================================
// 📌 Public function 3: sdlog_check_emergency_byte()
//    Call for every received byte in HAL_UART_RxCpltCallback's huart3 block.
//    Set g_emergencyAscent = 1 if 0xAA55+0xEA packet arrives.
// ============================================================
void sdlog_check_emergency_byte(uint8_t byte) {
    if (!emgActive) {
        if (byte == 0xAA) {          // Packet start
            emgActive = 1;
            emgLen = 0;
            emgBuf[emgLen++] = byte;
        }
        return;
    }

    emgBuf[emgLen++] = byte;

    if (emgLen >= sizeof(EmergencyCommand_t)) {
        EmergencyCommand_t* cmd = (EmergencyCommand_t*)emgBuf;
        if (cmd->header == 0xAA55 && cmd->command == 0xEA) {
            uint8_t expectedCrc = cmd->command ^ cmd->reason;
            if (cmd->checksum == expectedCrc) {
                g_emergencyAscent = 1;
                g_emergencyReason = cmd->reason;

                // Record emergency event to SD card
                if (sdMounted && fileOpened) {
                    char logLine[100];
                    int len = snprintf(logLine, sizeof(logLine),
                        "# EMERGENCY ASCENT at %lu ms, reason=0x%02X\r\n",
                        HAL_GetTick(), cmd->reason);
                    UINT bw;
                    f_write(&logFile, logLine, len, &bw);
                    f_sync(&logFile);
                }
            }
        }
        // Reset state (regardless of success/failure)
        emgActive = 0;
        emgLen = 0;
    }
}

// ============================================================
// 📌 Public function 4: sdlog_task()
//    Call inside while(1) loop. (Record status every 30 seconds)
// ============================================================
void sdlog_task(void) {
    static uint32_t lastStatusTime = 0;
    if (HAL_GetTick() - lastStatusTime > 30000) {
        lastStatusTime = HAL_GetTick();
        if (sdMounted && fileOpened) {
            char statusLine[80];
            int len = snprintf(statusLine, sizeof(statusLine),
                "# STATUS: lines=%lu, emergency=%d\r\n",
                lineCount, g_emergencyAscent);
            UINT bw;
            f_write(&logFile, statusLine, len, &bw);
            f_sync(&logFile);
        }
    }
}
