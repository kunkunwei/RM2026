// Created by kun on 25-7-12.
// Updated on 2025-07-13 for enhanced functionality and RTOS compatibility.

#ifndef EMM42_MOTOR_UART_H
#define EMM42_MOTOR_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Motor response status for UART communication
 */
typedef enum
{
  RESPONSE_OK,     // Command executed successfully (response: addr + 02 + 6B)
  RESPONSE_ERROR,  // Command error (response: addr + EE + 6B)
  RESPONSE_INVALID // Invalid response or communication failure
} Motor_ResponseStatus;

/**
 * @brief Motor addresses (1–247, 0 for broadcast)
 */
typedef enum
{
  EMM42_MOTOR_1_ADDR = 0x01, // Motor 1 address
  EMM42_MOTOR_2_ADDR = 0x02  // Motor 2 address
} EMM42_MOTOR_ADDR;


// 回传数据类型枚举
typedef enum {
  MOTOR_FEEDBACK_NONE = 0,
  MOTOR_FEEDBACK_POSITION,
  MOTOR_FEEDBACK_ERROR,
  MOTOR_FEEDBACK_ENABLE,
  MOTOR_FEEDBACK_STALL,
  MOTOR_FEEDBACK_CMD_ACK,    // 命令应答
  MOTOR_FEEDBACK_POS_DONE    // 位置命令完成
} Motor_FeedbackType;

// 回传数据结构体
typedef struct {
  Motor_FeedbackType type;
  uint8_t addr;
  int32_t position;
  int16_t error;
  uint8_t enable;
  uint8_t stall;
  bool ready;
} Motor_FeedbackData;


extern Motor_FeedbackData yaw_motor_feedback,pitch_motor_feedback;
// 解析函数
bool Motor_ParseFeedback(const uint8_t *rx, uint8_t len, Motor_FeedbackData *data);

// === Control Commands ===

/**
 * @brief Enable or disable the motor
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param enable true to enable, false to disable
 * @retval true Success, false Failure
 */
bool Motor_Enable(uint8_t addr, bool enable);

/**
 * @brief Set motor subdivision (1–256)
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param subdiv Subdivision value (1–256)
 * @retval true Success, false Failure or invalid subdiv
 */
bool Motor_SetSubdivision(uint8_t addr, uint8_t subdiv);

/**
 * @brief Unlock stall protection
 * @param addr Motor address (0–247, 0 for broadcast)
 * @retval true Success, false Failure
 */
bool Motor_UnlockStallProtection(uint8_t addr);

/**
 * @brief Set speed and acceleration for continuous rotation
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param speed Signed speed (-1279 to 1279); positive for clockwise, negative for counterclockwise
 * @param acc Acceleration (0–255; 255 disables curve acceleration)
 * @retval true Success, false Failure or invalid parameters
 */
bool Motor_SetSpeedDirection(uint8_t addr, int16_t speed, uint8_t acc);

/**
 * @brief Control relative movement by angle
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param speed Signed speed (-1279 to 1279); positive for clockwise, negative for counterclockwise
 * @param acc Acceleration (0–255; 255 disables curve acceleration)
 * @param angle Relative angle in degrees (converted to pulses internally)
 * @retval true Success, false Failure or invalid parameters
 */
bool Motor_MoveRelative(uint8_t addr, int16_t speed, uint8_t acc, float angle);

/**
 * @brief Emergency stop (immediate stop with max acceleration)
 * @param addr Motor address (0–247, 0 for broadcast)
 * @retval true Success, false Failure
 */
bool Motor_EmergencyStop(uint8_t addr);

// === Status Read Commands ===

/**
 * @brief Read encoder value
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param value Pointer to store encoder value (0–65535)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadEncoder(uint8_t addr, uint16_t *value);

/**
 * @brief Read current position
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param pos Pointer to store position (-2147483647 to 2147483647)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadPosition(uint8_t addr, int32_t *pos);

/**
 * @brief Read input pulse count
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param count Pointer to store pulse count (-2147483647 to 2147483647)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadPulseCount(uint8_t addr, int32_t *count);

/**
 * @brief Read position error
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param error Pointer to store error (-32767 to 32767)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadError(uint8_t addr, int16_t *error);

/**
 * @brief Read enable status
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param status Pointer to store status (0: disabled, 1: enabled)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadEnableStatus(uint8_t addr, uint8_t *status);

/**
 * @brief Read stall flag
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param flag Pointer to store stall flag (0: no stall, 1: stalled)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadStallFlag(uint8_t addr, uint8_t *flag);

/**
 * @brief Read auto-home status on power-up
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param status Pointer to store status (0: normal, 1: abnormal)
 * @retval true Success, false Failure or invalid response
 */
bool Motor_ReadAutoHomeStatus(uint8_t addr, uint8_t *status);

/**
 * @brief Set UART address
 * @param addr Current motor address (0–247, 0 for broadcast)
 * @param new_addr New address (1–247)
 * @retval true Success, false Failure or invalid new_addr
 */
bool Motor_SetUartAddress(uint8_t addr, uint8_t new_addr);

/**
 * @brief Store speed mode parameters for auto-run on power-up
 * @param addr Motor address (0–247, 0 for broadcast)
 * @retval true Success, false Failure
 */
bool Motor_StoreSpeedParameters(uint8_t addr);

/**
 * @brief Clear stored speed mode parameters
 * @param addr Motor address (0–247, 0 for broadcast)
 * @retval true Success, false Failure
 */
bool Motor_ClearSpeedParameters(uint8_t addr);

/**
 * @brief Convert position to angle in degrees
 * @param pos Position value (-2147483647 to 2147483647)
 * @return Angle in degrees
 */
float Motor_PositionToAngle(int32_t pos);

/**
 * @brief Convert error to angle in degrees
 * @param error Error value (-32767 to 32767)
 * @return Angle in degrees
 */
float Motor_ErrorToAngle(int16_t error);

#endif // EMM42_MOTOR_UART_H