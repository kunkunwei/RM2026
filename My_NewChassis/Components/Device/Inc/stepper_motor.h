//
// Created by kun on 25-7-27.
//

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : stepper_motor.h
  * @brief          : Stepper motor data structure and interface
  * @author         : Your Name
  * @date           : 2024/06/10
  * @version        : v1.0
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "stdint.h"
#include "stdbool.h"

typedef struct {
    uint8_t frame_id;      // CAN帧ID
    bool enabled;          // 使能状态
    int32_t position;      // 电机位置
    int32_t pulse_count;   // 输入脉冲数
    int16_t encoder;       // 编码器值
    int16_t angle;       // 角度值
    int16_t position_error;// 位置误差
    uint8_t stall_flag;    // 堵转标志
    float step_angle;      // 步距角，单位：度
    uint16_t subdiv;       // 细分数
} StepperMotor_Info_t;

extern StepperMotor_Info_t StepperMotor[2];

void StepperMotor_Init(StepperMotor_Info_t *motor, uint8_t frame_id);
// void StepperMotor_Update(StepperMotor_Info_t *motor, uint8_t *rx_data, uint8_t len);

#endif // STEPPER_MOTOR_H
