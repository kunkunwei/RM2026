/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepper_can.h
  * @brief          : New version stepper motor CAN protocol interface
  * @author         : [Your Name]
  * @date           : 2024/06/10
  * @version        : v2.0
  ******************************************************************************
  * @attention      : CAN communication uses extended frames with packet splitting
  *                  for position control commands.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef STEPPER_CAN_H
#define STEPPER_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stepper_motor.h"
#include "cmsis_os.h"

/* Constants */
#define CAN_CHECK_CODE          0x6B
#define CAN_MAX_RETRIES         2
#define CAN_RETRY_DELAY_MS      5
#define CAN_PACKET_DELAY_MS     2
#define CAN_SEM_WAIT_TIMEOUT    100

/* Exported types ------------------------------------------------------------*/
/* Motor IDs - Extended Frame IDs */
typedef enum {
    CAN1_YAW_MOTOR_ID   = 0x100,    // YAW轴电机扩展帧ID
    CAN1_PITCH_MOTOR_ID = 0x200     // PITCH轴电机扩展帧ID
} StepperMotorID_e;

/* Motor Info Structure */
typedef struct {
    StepperMotorID_e frame_id;
    float step_angle;
    uint16_t subdiv;
    volatile bool ready;
    float current_angle;     // 当前绝对角度
    bool position_valid;     // 位置数据是否有效
} StepperMotor_Info_t;

/* Exported variables --------------------------------------------------------*/
extern StepperMotor_Info_t yaw_motor_Info, pitch_motor_Info;

/* Exported functions --------------------------------------------------------*/
/* 核心控制函数 */
bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len);
bool StepperCAN_SendPositionCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len);

/* 电机基本控制 */
void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable);
void StepperCAN_SetAbsolutePosition(StepperMotor_Info_t *motor, float target_angle, uint16_t speed, uint8_t accel);
void StepperCAN_EmergencyStop(StepperMotorID_e motor_id);
void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode);

/* 位置读取 */
bool StepperCAN_ReadPosition(StepperMotorID_e motor_id);

/* 删除的不必要函数：
 * - StepperCAN_SetSpeed (不需要速度控制)
 * - StepperCAN_SetSingleZero (简化零点设置)
 * - StepperCAN_ReadSystemStatus (减少系统负担)
 * - 各种状态读取函数 (只保留位置读取)
 */

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_CAN_H */