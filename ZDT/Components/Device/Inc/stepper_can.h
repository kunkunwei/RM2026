/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepper_can.h
  * @brief          : Stepper motor CAN protocol interface
  * @author         : [Your Name]
  * @date           : 2024/06/10
  * @version        : v1.0
  ******************************************************************************
  * @attention      : CAN communication uses extended frames. Commands ≤ 8 bytes
  *                  are sent as a single frame. Commands > 8 bytes are split into
  *                  multiple packets with proper frame ID encoding.
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
#define CAN_MAX_RETRIES         3
#define CAN_RETRY_DELAY_MS      1
#define CAN_PACKET_DELAY_MS     10
#define CAN_SEM_WAIT_TIMEOUT    20

/* Exported types ------------------------------------------------------------*/
    /* Motor IDs */
    typedef enum {
        CAN1_YAW_MOTOR_ID   = 0x001,
        CAN1_PITCH_MOTOR_ID = 0x002
    } StepperMotorID_e;

    /* Motor Info Structure */
    typedef struct {
        StepperMotorID_e frame_id;
        float step_angle;
        uint16_t subdiv;
        volatile bool ready; // 新增，表示电机是否可发送新命令
    } StepperMotor_Info_t;

/* Exported variables --------------------------------------------------------*/
extern StepperMotor_Info_t yaw_motor_Info, pitch_motor_Info;
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

/* Exported functions --------------------------------------------------------*/

    /* Function Prototypes */
    bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len);
    void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable);
    void StepperCAN_SetSpeed(StepperMotor_Info_t *motor, int16_t speed, uint8_t accel);
    void StepperCAN_SetPositionAngle(StepperMotor_Info_t *motor, float angle, uint16_t speed, uint8_t accel, uint8_t abs_mode);
    void StepperCAN_EmergencyStop(StepperMotorID_e motor_id);
    void StepperCAN_SetSingleZero(StepperMotorID_e motor_id, uint8_t store_flag);
    void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode);
    void StepperCAN_ReadSystemStatus(StepperMotorID_e motor_id);
    void StepperCAN_SetZero(StepperMotorID_e motor_id);
    void StepperCAN_UnlockStallProtection(StepperMotorID_e motor_id);

    bool StepperCAN_ReadEncoder(StepperMotorID_e motor_id, uint16_t *value);
    bool StepperCAN_ReadPulseCount(StepperMotorID_e motor_id, int32_t *count);
    bool StepperCAN_ReadPosition(StepperMotorID_e motor_id, int32_t *pos);
    bool StepperCAN_ReadError(StepperMotorID_e motor_id, int16_t *error);
    bool StepperCAN_ReadEnableStatus(StepperMotorID_e motor_id, uint8_t *status);
    bool StepperCAN_ReadStallFlag(StepperMotorID_e motor_id, uint8_t *flag);
    bool StepperCAN_ReadAutoHomeStatus(StepperMotorID_e motor_id, uint8_t *status);
#ifdef __cplusplus
}
#endif

#endif /* STEPPER_CAN_H */