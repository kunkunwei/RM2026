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
// #include "freertos_config.h"

  /* Constants */
#define CAN_CHECK_CODE          0x6B
#define CAN_MAX_RETRIES         3
#define CAN_RETRY_DELAY_MS      1
#define CAN_PACKET_DELAY_MS     10
#define CAN_SEM_WAIT_TIMEOUT    20

/* Exported types ------------------------------------------------------------*/
    /* Motor IDs */
    typedef enum {
        CAN1_YAW_MOTOR_ID   = 0x100,
        CAN1_PITCH_MOTOR_ID = 0x200
    } StepperMotorID_e; /**< 电机帧 ID，使用扩展帧范围 */

    /* Motor Info Structure */
    typedef struct {
        StepperMotorID_e frame_id; /**< 电机帧 ID */
        float step_angle; /**< 单步角度（度） */
        uint16_t subdiv; /**< 驱动细分（每步细分数） */
    } StepperMotor_Info_t;

/* Exported variables --------------------------------------------------------*/
extern StepperMotor_Info_t yaw_motor_Info, pitch_motor_Info;

/* Exported functions --------------------------------------------------------*/

    /* Function Prototypes */
    /**
 * @brief 通过 CAN 向指定电机发送命令（支持单/多帧）
 * @param motor_id 电机帧 ID，如 CAN1_YAW_MOTOR_ID
 * @param cmd 指向命令数据
 * @param len 命令长度（最大 16）
 * @return true 表示收到了应答/发送成功，false 表示参数错误或超时
 */
    bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len);

    /**
 * @brief 使能或失能电机
 */
    void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable);

    /**
 * @brief 设置电机速度
 */
    void StepperCAN_SetSpeed(StepperMotor_Info_t *motor, int16_t speed, uint8_t accel);

    /**
 * @brief 按角度定位
 */
    void StepperCAN_SetPositionAngle(StepperMotor_Info_t *motor, float angle, uint16_t speed, uint8_t accel, uint8_t abs_mode);

    /**
 * @brief 紧急停止
 */
    void StepperCAN_EmergencyStop(StepperMotorID_e motor_id);

    /**
 * @brief 设置并可存储单圈零点
 */
    void StepperCAN_SetSingleZero(StepperMotorID_e motor_id, uint8_t store_flag);

    /**
 * @brief 触发回零
 */
    void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode);

    /**
 * @brief 请求读取电机系统状态（多帧返回）
 */
    void StepperCAN_ReadSystemStatus(StepperMotorID_e motor_id);
#ifdef __cplusplus
}
#endif

#endif /* STEPPER_CAN_H */

