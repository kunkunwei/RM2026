// Created by kun on 25-7-12.
// Updated on 2025-07-13 for FreeRTOS V10.x and simplified PWM position control.

#ifndef STEPPER_PWM_CONTROL_H
#define STEPPER_PWM_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "emm42_motor_uart.h"

/**
 * @brief Stepper motor control mode
 */
typedef enum {
    STEPPER_MODE_IDLE = 0,    // Motor idle
    STEPPER_MODE_SPEED,       // Speed mode (continuous rotation)
    STEPPER_MODE_POSITION,    // Position mode (fixed pulses)
    STEPPER_MODE_STOPPING     // Stopping (decelerating or emergency stop)
} StepperMode;

/**
 * @brief Motor measurement structure
 */
typedef struct {
    uint16_t encoder;           // Encoder value (0–65535)
    int32_t pos;                // Current position (-2147483647 to 2147483647)
    int32_t last_pos;
    uint32_t tick;              // Current tick count (ms)
    uint32_t last_tick;         // Previous tick count (ms)
    int32_t pulse_count;        // Input pulse count (-2147483647 to 2147483647)
    int16_t error;              // Position error (-32767 to 32767)
    uint8_t enable_status;      // Enable status (0: disabled, 1: enabled)
    uint8_t stall_flag;         // Stall flag (0: no stall, 1: stalled)
    uint8_t auto_home_status;   // Auto-home status (0: normal, 1: abnormal)
    uint8_t position_error;     // Position control error (0: none, 1: UART failure, 2: timeout, 3: stall)
    float angle;                // Position in degrees
    float error_angle;          // Error in degrees
    float angular_velocity;     // Angular velocity (rad/s)
    float target_speed;         // Target speed (steps/s)
    float accel;                // Acceleration (steps/s²)
    uint32_t target_pulses;     // Target pulses for position mode
    uint32_t sent_pulses;       // Pulses sent in position mode
    StepperMode mode;           // Current control mode (STEPPER_MODE_IDLE, STEPPER_MODE_SPEED, etc.)
} emm42_motor_measure_t;

extern emm42_motor_measure_t stepper_motor[2];
extern SemaphoreHandle_t motor_mutex;

/**
 * @brief Initialize PWM, direction pins, and mutex for motor control
 * @note Must be called after HAL and FreeRTOS initialization
 */
void Stepper_Init(void);

/**
 * @brief Set motor speed (continuous rotation)
 * @param motor_id Motor ID (0 or 1, maps to EMM42_MOTOR_1_ADDR or EMM42_MOTOR_2_ADDR)
 * @param speed Signed speed (-1279 to 1279 steps/s); positive for clockwise, negative for counterclockwise
 * @param accel Acceleration (0–255 steps/s²; 255 for no acceleration curve)
 */
void Stepper_SetSpeed(uint8_t motor_id, int16_t speed, uint8_t accel);

/**
 * @brief Move motor by a relative angle by sending a fixed number of pulses
 * @param motor_id Motor ID (0 or 1)
 * @param speed Signed speed (-1279 to 1279 steps/s); positive for clockwise, negative for counterclockwise
 * @param accel Acceleration (0–255 steps/s²; 255 for no acceleration curve)
 * @param angle Relative angle in degrees
 */
void Stepper_MoveRelative(uint8_t motor_id, int16_t speed, uint8_t accel, float angle);

/**
 * @brief Emergency stop (immediate stop)
 * @param motor_id Motor ID (0 or 1)
 */
void Stepper_EmergencyStop(uint8_t motor_id);

/**
 * @brief Update motor state machine and PWM output
 * @note Call periodically in a FreeRTOS task (e.g., every 10 ms)
 */
void Stepper_Update(void);

/**
 * @brief Get current motor mode
 * @param motor_id Motor ID (0 or 1)
 * @return Current mode (STEPPER_MODE_IDLE, STEPPER_MODE_SPEED, etc.)
 */
StepperMode Stepper_GetMode(uint8_t motor_id);

/**
 * @brief Update motor measurements (thread-safe)
 * @param motor_measure Pointer to measurement structure
 * @param motor_id Motor ID (0 or 1)
 * @note Requires initialized UART and motor_mutex
 */
void get_emm42_motor_measure(emm42_motor_measure_t *motor_measure, uint8_t motor_id);

/**
 * @brief Get pointer to motor measurement structure (thread-safe)
 * @param motor_id Motor ID (0 or 1)
 * @return Pointer to measurement structure
 */
emm42_motor_measure_t* get_emm42_motor_measure_point(uint8_t motor_id);

/**
 * @brief Timer interrupt callback for counting PWM pulses
 * @param htim Timer handle
 * @note Must be called from HAL_TIM_PeriodElapsedCallback
 */
void Stepper_PulseCallback(TIM_HandleTypeDef *htim);

#endif //STEPPER_PWM_CONTROL_H