/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepper_can.h
  * @brief          : Stepper motor CAN protocol interface
  * @author         : Your Name
  * @date           : 2024/06/10
  * @version        : v1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef STEPPER_CAN_H
#define STEPPER_CAN_H

#include "stdint.h"
#include "stepper_motor.h"

#define STEPPER_CAN_CHECK_CODE 0x6B

// 读取参数命令
void StepperCAN_ReadEncoder(uint8_t frame_id);
void StepperCAN_ReadPulseCount(uint8_t frame_id);
void StepperCAN_ReadPosition(uint8_t frame_id);
void StepperCAN_ReadPositionError(uint8_t frame_id);
void StepperCAN_ReadEnableState(uint8_t frame_id);
void StepperCAN_ReadStallFlag(uint8_t frame_id);
void StepperCAN_ReadZeroFlag(uint8_t frame_id);

void StepperMotor_UpdatePosition(StepperMotor_Info_t *motor, uint8_t *rx_data);

// 修改参数命令
void StepperCAN_SetSubdivision(uint8_t frame_id, uint8_t subdiv);
void StepperCAN_SetSerialAddr(uint8_t frame_id, uint8_t addr);

// 运动控制命令
void StepperCAN_Enable(uint8_t frame_id, bool enable);
void StepperCAN_Stop(uint8_t frame_id);

void StepperCAN_SetSpeed(uint8_t frame_id, uint16_t speed, uint8_t direction, uint8_t accel);
void StepperCAN_StoreSpeedParam(uint8_t frame_id, bool store); // store=true存储，false清除
void StepperCAN_RelativeMoveAngle(StepperMotor_Info_t *motor, uint16_t speed,uint8_t accel, float angle);

void StepperCAN_SendCommand(uint8_t frame_id, uint8_t *cmd, uint8_t len);
void tset_can();
#endif // STEPPER_CAN_H