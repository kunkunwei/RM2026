/**
 ******************************************************************************
 * @file           : system_monitor.c
 * @brief          : 系统监控实现
 * @author         :
 * @date           : 2026/01/05
 * @version        : v2.0
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "monitor.h"
#include "main.h" // 用于 DWT_GetTimeline_ms()
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "usart.h"
#include "vofa.h"

/* 外部 IWDG 句柄 (如果不可用，注释掉 Refresh 调用) */
// extern IWDG_HandleTypeDef hiwdg;

/* Private defines -----------------------------------------------------------*/
#define WATCHDOG_TIMEOUT_MS   1500  // 看门狗超时阈值 (1500ms)
#define COMM_TIMEOUT_MS       100  // 通信超时阈值 (100ms)
#define CPU_WINDOW_MS         1000 // CPU 负载计算窗口 (1 秒)
#define CAN_SILENT_TIMEOUT_MS 500  // CAN 沉默阈值 (500ms 无接收认为沉默)
#define BLINK_SLOW_PERIOD_MS  1000 // 慢闪周期 0.5 Hz = 每 1 秒一个完整周期
#define BLINK_FAST_PERIOD_MS  500  // 快闪周期 2 Hz = 每 0.5 秒一个完整周期

/* Private types -------------------------------------------------------------*/
// 已在头文件中定义 SystemMonitor_t

/* Private variables ---------------------------------------------------------*/
static SystemMonitor_t g_monitor           = {0}; // 全局监控器实例
static float g_last_heartbeat_warning_time = 0;
// CPU占用率计算的静态变量（轻量级方法）
static uint32_t cpu_last_idle_time = 0;    // 上次空闲任务运行时间
static uint32_t cpu_last_total_time = 0;   // 上次总时间（DWT周期数）


/* ============================================================================
 * 阶段1: 看门狗和心跳监控实现
 * ========================================================================== */

void Monitor_Heartbeat_Init(void)
{
    // 检查是否由看门狗重启
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        g_monitor.heartbeat.status = 2; // 错误
        g_monitor.heartbeat.watchdog_restart_count++;
        g_monitor.heartbeat.restart_reason = 1; // 看门狗重启
        __HAL_RCC_CLEAR_RESET_FLAGS();
    } else {
        g_monitor.heartbeat.status         = 0; // 正常
        g_monitor.heartbeat.restart_reason = 0;
    }

    g_monitor.heartbeat.heartbeat_count     = 0;
    g_monitor.heartbeat.last_heartbeat_time = DWT_GetTimeline_ms();
}

void Monitor_Heartbeat_Beat(void)
{
    float now = DWT_GetTimeline_ms();

    // 递增计数器
    g_monitor.heartbeat.heartbeat_count++;
    g_monitor.heartbeat.last_heartbeat_time = now;

    // 检查停滞 (每 2ms 调用，50 次未增长 = 100ms 停滞)
    static uint32_t missed_beats    = 0;
    static uint32_t last_beat_count = 0;

    if (g_monitor.heartbeat.heartbeat_count == last_beat_count) {
        missed_beats++;
    } else {
        missed_beats    = 0;
        last_beat_count = g_monitor.heartbeat.heartbeat_count;
    }

    if (missed_beats > 50) { // > 100ms 停滞
        if (g_monitor.heartbeat.status == 0) {
            g_monitor.heartbeat.status    = 1; // 警告
            g_last_heartbeat_warning_time = now;
        }
    } else {
        g_monitor.heartbeat.status = 0; // 正常
    }
}

void Monitor_Watchdog_Refresh(void)
{
    // 刷新 IWDG 以防止超时
    // HAL_IWDG_Refresh(&hiwdg);
}

Monitor_Heartbeat_t *Monitor_GetHeartbeat(void)
{
    return &g_monitor.heartbeat;
}

uint32_t Monitor_GetRestartCount(void)
{
    return g_monitor.heartbeat.watchdog_restart_count;
}

uint8_t Monitor_GetHeartbeatStatus(void)
{
    return g_monitor.heartbeat.status;
}

/* ============================================================================
 * PHASE 2: CAN BUS MONITOR IMPLEMENTATION
 * ========================================================================== */

void Monitor_CAN_Init(void)
{
    memset(&g_monitor.can, 0, sizeof(Monitor_CAN_t));
    g_monitor.can.status       = 0; // Normal (no data yet, not an error)
    g_monitor.can.last_rx_time = DWT_GetTimeline_ms();
}

void Monitor_CAN_RecordTx(void)
{
    g_monitor.can.tx_count++;
}

void Monitor_CAN_RecordRx(void)
{
    g_monitor.can.rx_count++;
    g_monitor.can.last_rx_time = DWT_GetTimeline_ms();
}

void Monitor_CAN_RecordError(uint8_t is_tx)
{
    if (is_tx) {
        g_monitor.can.tx_error_count++;
    } else {
        g_monitor.can.rx_error_count++;
    }
}

uint8_t Monitor_GetCANStatus(void)
{
    float now                = DWT_GetTimeline_ms();
    float time_since_last_rx = now - g_monitor.can.last_rx_time;

    // If no RX for too long, CAN is silent
    if (time_since_last_rx > CAN_SILENT_TIMEOUT_MS && g_monitor.can.rx_count > 0) {
        g_monitor.can.status = 2; // Silent
        return 2;
    }

    // If there are errors, status is warning
    if (g_monitor.can.tx_error_count > 0 || g_monitor.can.rx_error_count > 0) {
        g_monitor.can.status = 1; // Warning
        return 1;
    }

    g_monitor.can.status = 0; // Normal
    return 0;
}

Monitor_CAN_t *Monitor_GetCAN(void)
{
    return &g_monitor.can;
}

/* ============================================================================
 * PHASE 3: COMMUNICATION TIMEOUT MONITOR IMPLEMENTATION
 * ========================================================================== */

void Monitor_Comm_Init(void)
{
    memset(&g_monitor.comm, 0, sizeof(Monitor_Comm_t));
    g_monitor.comm.timeout_threshold_ms = COMM_TIMEOUT_MS;
    g_monitor.comm.last_rx_time         = DWT_GetTimeline_ms();
    g_monitor.comm.status               = 0;
}

void Monitor_Comm_RecordRx(void)
{
    g_monitor.comm.last_rx_time         = DWT_GetTimeline_ms();
    g_monitor.comm.consecutive_timeouts = 0;
    g_monitor.comm.status               = 0;
}

uint8_t Monitor_GetCommStatus(void)
{
    float now           = DWT_GetTimeline_ms();
    float time_since_rx = now - g_monitor.comm.last_rx_time;

    if (time_since_rx > g_monitor.comm.timeout_threshold_ms) {
        g_monitor.comm.consecutive_timeouts++;
        g_monitor.comm.status = 2; // Timeout
        return 2;
    }

    if (time_since_rx > g_monitor.comm.timeout_threshold_ms * 0.8f) {
        g_monitor.comm.status = 1; // Late
        return 1;
    }

    g_monitor.comm.status = 0; // OK
    return 0;
}

Monitor_Comm_t *Monitor_GetComm(void)
{
    return &g_monitor.comm;
}

/* ============================================================================
 * PHASE 4: CPU LOAD MONITOR IMPLEMENTATION
 * ========================================================================== */

void Monitor_CPU_Init(void)
{
    memset(&g_monitor.cpu, 0, sizeof(Monitor_CPU_t));
    g_monitor.cpu.window_start_time  = DWT_GetTimeline_ms();
    g_monitor.cpu.window_duration_ms = CPU_WINDOW_MS;
    g_monitor.cpu.cpu_load_percent   = 0;
    g_monitor.cpu.status             = 0;

    // 初始化统计变量
    cpu_last_idle_time = 0;
    cpu_last_total_time = 0;
}

void Monitor_CPU_RecordLoop(void)
{
    float now     = DWT_GetTimeline_ms();
    float elapsed = now - g_monitor.cpu.window_start_time;

    // 每1秒计算一次CPU占用率
    if (elapsed >= g_monitor.cpu.window_duration_ms) {
        TaskStatus_t *pxTaskStatusArray;
        UBaseType_t uxArraySize, x;
        uint32_t ulTotalRunTime, ulIdleRunTime = 0;

        // 获取任务数量
        uxArraySize = uxTaskGetNumberOfTasks();

        // 分配任务状态数组
        pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

        if (pxTaskStatusArray != NULL) {
            // 获取所有任务的运行时统计
            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
            // uart_printf(&huart6,"DEBUG: ulTotalRunTime=%lu, TaskCount=%u\r\n", ulTotalRunTime, uxArraySize);
            // 查找IDLE任务（优先级为0）
            for (x = 0; x < uxArraySize; x++) {
                if (pxTaskStatusArray[x].uxCurrentPriority == 0) {
                    ulIdleRunTime = pxTaskStatusArray[x].ulRunTimeCounter;
                    break;
                }
            }

            // 释放内存
            vPortFree(pxTaskStatusArray);

            // 计算CPU占用率（基于增量）
            // if (ulTotalRunTime > cpu_last_total_time) {
                uint32_t delta_total = ulTotalRunTime - cpu_last_total_time;
                uint32_t delta_idle = ulIdleRunTime - cpu_last_idle_time;
                // uart_printf(&huart6,"DEBUG: delta_total=%lu, delta_idle=%lu\r\n", delta_total, delta_idle);
                // 第一次调用时跳过（delta值不可靠）
                if (cpu_last_total_time > 0 && delta_total > 0) {
                    float cpu_usage = 100.0f * (1.0f - (float)delta_idle / delta_total);
                    if (cpu_usage < 0.0f) cpu_usage = 0.0f;
                    if (cpu_usage > 100.0f) cpu_usage = 100.0f;

                    g_monitor.cpu.cpu_load_percent = (uint8_t)cpu_usage;
                    // uart_printf(&huart6,"DEBUG: CPU usage = %d%%\r\n", g_monitor.cpu.cpu_load_percent);
                }

                // 保存当前值
                cpu_last_total_time = ulTotalRunTime;
                cpu_last_idle_time = ulIdleRunTime;
            // }
        }

        // 更新状态
        if (g_monitor.cpu.cpu_load_percent > 85) {
            g_monitor.cpu.status = 2; // Overload
        } else if (g_monitor.cpu.cpu_load_percent > 70) {
            g_monitor.cpu.status = 1; // High
        } else {
            g_monitor.cpu.status = 0; // Normal
        }

        // 重置时间窗口
        g_monitor.cpu.window_start_time = now;
    }
}

uint8_t Monitor_GetCPULoad(void)
{
    return g_monitor.cpu.cpu_load_percent;
}

uint8_t Monitor_GetCPUStatus(void)
{
    return g_monitor.cpu.status;
}

Monitor_CPU_t *Monitor_GetCPU(void)
{
    return &g_monitor.cpu;
}

/* ============================================================================
 * LED CONTROL IMPLEMENTATION
 * ========================================================================== */

// LED pin definitions (from main.h)
// #define LED_R_Pin       GPIO_PIN_12
// #define LED_R_GPIO_Port GPIOH
// #define LED_G_Pin       GPIO_PIN_11
// #define LED_G_GPIO_Port GPIOH
// #define LED_B_Pin       GPIO_PIN_10
// #define LED_B_GPIO_Port GPIOH

static void led_set(uint8_t led, uint8_t on)
{
    GPIO_TypeDef *port = NULL;
    uint16_t pin       = 0;

    if (led == 0) { // Red
        port = LED_R_GPIO_Port;
        pin  = LED_R_Pin;
    } else if (led == 1) { // Green
        port = LED_G_GPIO_Port;
        pin  = LED_G_Pin;
    } else if (led == 2) { // Blue
        port = LED_B_GPIO_Port;
        pin  = LED_B_Pin;
    }

    if (port != NULL) {
        HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void led_blink(uint8_t led, uint8_t mode)
{
    float now     = DWT_GetTimeline_ms();
    uint8_t state = 0;

    if (mode == 0) { // Off
        state = 0;
    } else if (mode == 1) { // On
        state = 1;
    } else if (mode == 2) { // Slow Blink (0.5Hz)
        state = (((uint32_t)now / BLINK_SLOW_PERIOD_MS) % 2) == 0 ? 1 : 0;
    } else if (mode == 3) { // Fast Blink (2Hz)
        state = (((uint32_t)now / BLINK_FAST_PERIOD_MS) % 2) == 0 ? 1 : 0;
    }

    led_set(led, state);
}

Monitor_LED_State_t *Monitor_GetLEDState(void)
{
    return &g_monitor.led;
}

void Monitor_SetLED(uint8_t led, uint8_t mode)
{
    if (led == 0) {
        g_monitor.led.red_mode = mode;
    } else if (led == 1) {
        g_monitor.led.green_mode = mode;
    } else if (led == 2) {
        g_monitor.led.blue_mode = mode;
    }
}

void Monitor_UpdateLEDs(void)
{
    // Red LED: Watchdog/Heartbeat errors
    // 0=Off, 1=On (error), 2=Slow Blink (warning)
    uint8_t hb_status = Monitor_GetHeartbeatStatus();
    if (hb_status == 2) {        // Error (WDG restart detected)
        Monitor_SetLED(0, 1);    // Red On
    } else if (hb_status == 1) { // Warning (stall detected)
        Monitor_SetLED(0, 2);    // Red Slow Blink
    } else {
        Monitor_SetLED(0, 0); // Red Off
    }

    // Green LED: Communication status
    // 0=Off, 1=On (OK), 2=Slow Blink (late), 3=Fast Blink (timeout)
    uint8_t comm_status = Monitor_GetCommStatus();
    if (comm_status == 0) {
        Monitor_SetLED(1, 1); // Green On
    } else if (comm_status == 1) {
        Monitor_SetLED(1, 2); // Green Slow Blink
    } else {
        Monitor_SetLED(1, 3); // Green Fast Blink
    }

    // Blue LED: CPU load
    // 0=Off, 1=On, 2=Slow Blink (high), 3=Fast Blink (overload)
    uint8_t cpu_status = Monitor_GetCPUStatus();
    if (cpu_status == 0) {
        Monitor_SetLED(2, 0); // Blue Off
    } else if (cpu_status == 1) {
        Monitor_SetLED(2, 2); // Blue Slow Blink
    } else {
        Monitor_SetLED(2, 3); // Blue Fast Blink
    }

    // Apply the blinking
    led_blink(0, g_monitor.led.red_mode);
    led_blink(1, g_monitor.led.green_mode);
    led_blink(2, g_monitor.led.blue_mode);
}

/* ============================================================================
 * MAIN MONITOR INTERFACE
 * ========================================================================== */

void Monitor_Init(void)
{
    Monitor_Heartbeat_Init();
    Monitor_CAN_Init();
    Monitor_Comm_Init();
    Monitor_CPU_Init();

    memset(&g_monitor.led, 0, sizeof(Monitor_LED_State_t));
}

void Monitor_Process(void)
{
    // Called every 2ms in control loop
    Monitor_Heartbeat_Beat();
    Monitor_CPU_RecordLoop();
}

void Monitor_PeriodicTask(void)
{
    // Called every ~50ms to update LEDs
    Monitor_UpdateLEDs();
}

uint8_t Monitor_GetSystemStatus(void)
{
    uint8_t max_status = 0;

    max_status = (Monitor_GetHeartbeatStatus() > max_status) ? Monitor_GetHeartbeatStatus() : max_status;
    max_status = (Monitor_GetCANStatus() > max_status) ? Monitor_GetCANStatus() : max_status;
    max_status = (Monitor_GetCommStatus() > max_status) ? Monitor_GetCommStatus() : max_status;
    max_status = (Monitor_GetCPUStatus() > max_status) ? Monitor_GetCPUStatus() : max_status;

    return max_status;
}
/* ============================================================================
 * 主监控器API实现
 * ========================================================================== */

SystemMonitor_t *SystemMonitor_Get(void)
{
    return &g_monitor;
}

void SystemMonitor_Init(void)
{
    Monitor_Heartbeat_Init();
    Monitor_CAN_Init();
    Monitor_Comm_Init();
    Monitor_CPU_Init();
    memset(&g_monitor.led, 0, sizeof(Monitor_LED_State_t));
}

void SystemMonitor_Process(void)
{
    // 每2ms调用：心跳更新 + CPU循环记录
    Monitor_Heartbeat_Beat();
    Monitor_CPU_RecordLoop();
}

void SystemMonitor_PeriodicTask(void)
{
    // 每50ms调用：更新LED
    Monitor_UpdateLEDs();
}

void SystemMonitor_WatchdogRefresh(void)
{
    Monitor_Watchdog_Refresh();
}

uint8_t SystemMonitor_GetStatus(void)
{
    return Monitor_GetSystemStatus();
}

void SystemMonitor_CAN_RecordTx(void)
{
    Monitor_CAN_RecordTx();
}

void SystemMonitor_CAN_RecordRx(void)
{
    Monitor_CAN_RecordRx();
}

void SystemMonitor_CAN_RecordError(uint8_t is_tx)
{
    Monitor_CAN_RecordError(is_tx);
}

void SystemMonitor_Comm_RecordRx(void)
{
    Monitor_Comm_RecordRx();
}