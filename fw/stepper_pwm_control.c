// Created by kun on 25-7-12.
// Updated on 2025-07-13 for FreeRTOS V10.x and simplified PWM position control.

/**
 * @file    stepper_pwm_control.c
 * @brief   PWM-based closed-loop control for Emm42_V4.2 stepper motor with speed and position modes
 * @version 2.5
 * @date    2025-07-13
 */

#include "stepper_pwm_control.h"
#include "tim.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "emm42_motor_uart.h"
#include "dma.h"
#include <math.h>
#include "stdio.h"
#include "usart.h"
#include "vofa.h"

/**
 * @brief Motor measurement array for two motors
 */
emm42_motor_measure_t stepper_motor[2] = {0};

/**
 * @brief Mutex for thread-safe access to stepper_motor
 */
SemaphoreHandle_t motor_mutex = NULL;

/**
 * @brief PWM control parameters
 */
#define MAX_PWM_FREQ 1279 // Max frequency (steps/s), matching UART speed limit
#define PULSES_PER_REV 3200 // Pulses per revolution (16 subdivisions, 1.8° motor)
#define POSITION_TIMEOUT_MS 10000 // Timeout for position mode (ms)
#define MIN_PULSE_WIDTH_US 2 // Minimum pulse width (us) for Emm42_V4.2
#define MAX_DMA_PULSES 4096

static uint16_t stepper_dma_buf[MAX_DMA_PULSES+1];
static bool dma_busy = false;

/**
 * @brief Get timer clock frequency (auto x2 if needed)
 * @param htim Pointer to TIM handle
 * @return Timer clock frequency in Hz
 */
static uint32_t Get_Timer_Clock(TIM_HandleTypeDef *htim) {
    uint32_t pclk, ppre;
    if (htim->Instance == TIM1 || htim->Instance == TIM8 || htim->Instance == TIM9 ||
        htim->Instance == TIM10 || htim->Instance == TIM11) {
        // APB2 timers
        pclk = HAL_RCC_GetPCLK2Freq();
        ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
        if (ppre >= 4) // If prescaler >= /2, timer clk = pclk*2
            return pclk * 2;
        else
            return pclk;
        } else {
            // APB1 timers
            pclk = HAL_RCC_GetPCLK1Freq();
            ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
            if (ppre >= 4)
                return pclk * 2;
            else
                return pclk;
        }
}
/**
 * @brief Initialize PWM, direction pins, and mutex
 * @note Uses htim1 (TIM_CHANNEL_1 for motor 0, TIM_CHANNEL_3 for motor 1)
 */
void Stepper_Init(void) {
    motor_mutex = xSemaphoreCreateMutex();
    if (motor_mutex == NULL) {
        // Handle mutex creation failure (e.g., log error)
        while (1); // Replace with proper error handling
    }


    // Set 16 subdivisions and enable pulse/direction mode via UART
    Motor_SetSubdivision(EMM42_MOTOR_1_ADDR, 16);
    Motor_SetSubdivision(EMM42_MOTOR_2_ADDR, 16);
    // 使用IO引脚失能电机（高电平使能）

    HAL_GPIO_WritePin(Motor1_en_GPIO_Port, Motor1_en_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor2_en_GPIO_Port, Motor2_en_Pin, GPIO_PIN_RESET);

    // Initialize PWM for both motors (enable PWM中断)
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1); // Motor 0
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3); // Motor 1
    HAL_GPIO_WritePin(Motor1_dir_GPIO_Port, Motor1_dir_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Motor2_dir_GPIO_Port, Motor2_dir_Pin, GPIO_PIN_RESET);
    stepper_motor[0].mode = STEPPER_MODE_IDLE;
    stepper_motor[1].mode = STEPPER_MODE_IDLE;

    // Enable timer interrupt for pulse counting
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim3);
}

/**
 * @brief Set PWM frequency for the specified motor
 * @param motor_id Motor ID (0 or 1)
 * @param frequency Desired frequency (steps/s)
 * @note Ensures minimum pulse width of 2us
 */
static void Stepper_SetFrequency(uint8_t motor_id, uint32_t frequency) {
    if (motor_id > 1) return;
    if (frequency > MAX_PWM_FREQ) frequency = MAX_PWM_FREQ;
    TIM_HandleTypeDef *htim = (motor_id == 0) ? &htim1 : &htim3;
    uint32_t channel = (motor_id == 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_3;

    if (frequency == 0) {
        HAL_TIM_PWM_Stop(htim, channel);
        return;
    }

    uint32_t timer_clk = Get_Timer_Clock(htim);
    uint32_t prescaler = 0;
    uint32_t period = 0;

    // Ensure pulse width >= 2us
    for (prescaler = 0; prescaler < 0xFFFF; prescaler++) {
        period = (timer_clk / (frequency * (prescaler + 1))) - 1;
        uint32_t pulse_width_us = (period + 1) * (prescaler + 1) * 1000000 / timer_clk;
        if (period < 0xFFFF && pulse_width_us >= MIN_PULSE_WIDTH_US * 2) break;
    }
    __HAL_TIM_SET_PRESCALER(htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(htim, period);
    __HAL_TIM_SET_COMPARE(htim, channel, period / 2); // 50% duty
    HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);

    HAL_TIM_PWM_Start(htim, channel); // (re)start if not running
}

/**
 * @brief Timer interrupt callback for counting PWM pulses
 * @param htim Timer handle
 */
void  Stepper_PulseCallback(TIM_HandleTypeDef *htim) {
    // if (htim->Instance != TIM1&&htim->Instance != TIM3) return;
    uint8_t motor_id;
    if (htim->Instance == TIM1) {
        motor_id = 0;
    } else if (htim->Instance == TIM3) {
        motor_id = 1;
    } else {
        return;
    }
    if (xSemaphoreTakeFromISR(motor_mutex, NULL) != pdTRUE) return;

    // for (uint8_t motor_id = 0; motor_id < 2; motor_id++) {
        if (stepper_motor[motor_id].mode == STEPPER_MODE_POSITION) {
            stepper_motor[motor_id].sent_pulses++;
            if (stepper_motor[motor_id].sent_pulses >= stepper_motor[motor_id].target_pulses) {
                // Stop PWM
                Stepper_SetFrequency(motor_id, 0);
                stepper_motor[motor_id].mode = STEPPER_MODE_IDLE;
                stepper_motor[motor_id].sent_pulses = 0;
                stepper_motor[motor_id].target_pulses = 0;
                Motor_Enable(motor_id + 1, false);
                HAL_GPIO_TogglePin(LED_R_GPIO_Port,LED_R_Pin);
            }
        }
    // }

    xSemaphoreGiveFromISR(motor_mutex, NULL);
}
// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
// {
//     Stepper_PulseCallback(htim);
// }
void Stepper_SetSpeed(uint8_t motor_id, int16_t speed, uint8_t accel) {
    if (motor_id > 1) return;
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return;

    // 使用IO引脚使能电机（高电平使能）
    GPIO_TypeDef *en_port = (motor_id == 0) ? Motor1_en_GPIO_Port : Motor2_en_GPIO_Port;
    uint16_t en_pin = (motor_id == 0) ? Motor1_en_Pin : Motor2_en_Pin;
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);

    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > MAX_PWM_FREQ) abs_speed = MAX_PWM_FREQ;
    bool direction = (speed >= 0);

    stepper_motor[motor_id].target_speed = abs_speed;
    stepper_motor[motor_id].accel = accel;
    stepper_motor[motor_id].mode = STEPPER_MODE_SPEED;
    stepper_motor[motor_id].sent_pulses = 0;
    stepper_motor[motor_id].target_pulses = 0;
    stepper_motor[motor_id].position_error = 0;

    GPIO_TypeDef *port = (motor_id == 0) ? Motor1_dir_GPIO_Port : Motor2_dir_GPIO_Port;
    uint16_t pin = (motor_id == 0) ? Motor1_dir_Pin : Motor2_dir_Pin;
    HAL_GPIO_WritePin(port, pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Stepper_SetFrequency(motor_id, abs_speed);

    xSemaphoreGive(motor_mutex);
}

void Stepper_MoveRelative(uint8_t motor_id, int16_t speed, uint8_t accel, float angle) {
    if (motor_id > 1 || fabs(angle) < 0.01f) {
        stepper_motor[motor_id].position_error = 1; // Invalid parameters
        return;
    }
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return;

    // 使用IO引脚使能电机（高电平使能）
    GPIO_TypeDef *en_port = (motor_id == 0) ? Motor1_en_GPIO_Port : Motor2_en_GPIO_Port;
    uint16_t en_pin = (motor_id == 0) ? Motor1_en_Pin : Motor2_en_Pin;
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_SET);

    bool direction = (angle >= 0);  //  用 angle 决定方向
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > MAX_PWM_FREQ) abs_speed = MAX_PWM_FREQ;
    if (abs_speed < 50) abs_speed = 50; // Minimum speed

    // 用 angle 的绝对值算目标脉冲
    uint32_t target_pulses = (uint32_t)(fabs(angle) * PULSES_PER_REV / 360.0f + 0.5f);  // 四舍五入更精准
    if (target_pulses == 0) {
        stepper_motor[motor_id].position_error = 1;
        xSemaphoreGive(motor_mutex);
        return;
    }
    stepper_motor[motor_id].target_pulses = target_pulses;
    stepper_motor[motor_id].target_speed = abs_speed;
    stepper_motor[motor_id].accel = accel;
    stepper_motor[motor_id].mode = STEPPER_MODE_POSITION;
    stepper_motor[motor_id].sent_pulses = 0;
    stepper_motor[motor_id].position_error = 0;
    stepper_motor[motor_id].tick = xTaskGetTickCount();

    // 设置方向引脚
    GPIO_TypeDef *port = (motor_id == 0) ? Motor1_dir_GPIO_Port : Motor2_dir_GPIO_Port;
    uint16_t pin = (motor_id == 0) ? Motor1_dir_Pin : Motor2_dir_Pin;
    HAL_GPIO_WritePin(port, pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

    Stepper_SetFrequency(motor_id, abs_speed);  // 一律用正速度

    xSemaphoreGive(motor_mutex);
}


void Stepper_EmergencyStop(uint8_t motor_id) {
    if (motor_id > 1) return;
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return;

    Stepper_SetFrequency(motor_id, 0); // Stop PWM
    stepper_motor[motor_id].mode = STEPPER_MODE_IDLE;
    stepper_motor[motor_id].sent_pulses = 0;
    stepper_motor[motor_id].target_pulses = 0;
    stepper_motor[motor_id].position_error = 0;
    // 使用IO引脚失能电机（低电平失能）
    GPIO_TypeDef *en_port = (motor_id == 0) ? Motor1_en_GPIO_Port : Motor2_en_GPIO_Port;
    uint16_t en_pin = (motor_id == 0) ? Motor1_en_Pin : Motor2_en_Pin;
    HAL_GPIO_WritePin(en_port, en_pin, GPIO_PIN_RESET);

    xSemaphoreGive(motor_mutex);
}

void Stepper_Update(void) {
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return;

    for (uint8_t motor_id = 0; motor_id < 2; motor_id++) {
        emm42_motor_measure_t *motor = &stepper_motor[motor_id];

        switch (motor->mode) {
            case STEPPER_MODE_IDLE:
                break;

            case STEPPER_MODE_SPEED:
                // Maintain speed, check for stall
                if (motor->stall_flag) {
                    Stepper_EmergencyStop(motor_id);
                    motor->position_error = 2; // Stall detected
                }
                break;

            case STEPPER_MODE_POSITION:
                // Check timeout
                uint32_t current_tick = xTaskGetTickCount();
                if ((current_tick - motor->tick) > pdMS_TO_TICKS(POSITION_TIMEOUT_MS)) {
                    Stepper_EmergencyStop(motor_id);
                    motor->position_error = 2; // Timeout
                }
                break;

            case STEPPER_MODE_STOPPING:
                Stepper_EmergencyStop(motor_id);
                break;
        }
    }

    xSemaphoreGive(motor_mutex);
}

StepperMode Stepper_GetMode(uint8_t motor_id) {
    if (motor_id > 1) return STEPPER_MODE_IDLE;
    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return STEPPER_MODE_IDLE;
    StepperMode mode = stepper_motor[motor_id].mode;
    xSemaphoreGive(motor_mutex);
    return mode;
}

void get_emm42_motor_measure(emm42_motor_measure_t *motor_measure, uint8_t motor_id) {
    if (motor_measure == NULL || motor_id > 1) return;

    if (xSemaphoreTake(motor_mutex, portMAX_DELAY) != pdTRUE) return;

    motor_measure->last_tick = motor_measure->tick;
    motor_measure->tick = xTaskGetTickCount();
    motor_measure->last_pos = motor_measure->pos;

    // Read motor parameters via UART
    Motor_ReadEncoder(motor_id + 1, &motor_measure->encoder);
    Motor_ReadPosition(motor_id + 1, &motor_measure->pos);
    Motor_ReadPulseCount(motor_id + 1, &motor_measure->pulse_count);
    Motor_ReadError(motor_id + 1, &motor_measure->error);
    Motor_ReadEnableStatus(motor_id + 1, &motor_measure->enable_status);
    Motor_ReadStallFlag(motor_id + 1, &motor_measure->stall_flag);
    Motor_ReadAutoHomeStatus(motor_id + 1, &motor_measure->auto_home_status);

    // Calculate derived parameters
    motor_measure->angle = Motor_PositionToAngle(motor_measure->pos);
    motor_measure->error_angle = Motor_ErrorToAngle(motor_measure->error);

    int32_t delta_pos = motor_measure->pos - motor_measure->last_pos;
    uint32_t delta_tick = motor_measure->tick - motor_measure->last_tick;
    float dt_s = (float)delta_tick / (1000.0f / configTICK_RATE_HZ);
    motor_measure->angular_velocity = (dt_s > 0) ? ((float)delta_pos * 2.0f * 3.1415926f / 65536.0f) / dt_s : 0.0f;

    xSemaphoreGive(motor_mutex);
}

emm42_motor_measure_t* get_emm42_motor_measure_point(uint8_t motor_id) {
    if (motor_id > 1) return NULL;
    return &stepper_motor[motor_id];
}