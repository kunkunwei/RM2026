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

/**
 * @brief 步进电机回零模式
 */
typedef enum {
    STEPPER_HOME_SINGLE_NEAREST = 0x00,    // 单圈就近回零
    STEPPER_HOME_SINGLE_DIR     = 0x01,    // 单圈方向回零
    STEPPER_HOME_MULTI_NO_LIMIT = 0x02,    // 多圈无限位碰撞回零
    STEPPER_HOME_MULTI_LIMIT    = 0x03     // 多圈有限位开关回零
  } StepperHomeMode_e;

/**
 * @brief 步进电机状态标志
 */
typedef enum {
    STEPPER_STATUS_ENABLED        = 0x01, // 电机使能
    STEPPER_STATUS_IN_POSITION    = 0x02, // 电机到位
    STEPPER_STATUS_STALL          = 0x04, // 电机堵转
    STEPPER_STATUS_STALL_PROTECT  = 0x08  // 电机堵转保护
  } StepperStatusFlag_e;

/**
 * @brief 步进电机测量数据结构
 */
typedef struct {
    uint8_t param_len;         // 返回字节数
    uint8_t param_count;       // 配置参数个数
    uint16_t bus_voltage;      // 总线电压(mV)
    uint16_t bus_current;      // 总线相电流(mA)
    uint16_t encoder_value;    // 校准后编码器值
    int32_t target_position;   // 电机目标位置(角度)
    int16_t realtime_speed;    // 实时转速(RPM)
    int32_t realtime_position; // 实时位置(角度)
    int32_t position_error;    // 位置误差(角度)
    uint8_t home_status;       // 回零状态标志
    StepperStatusFlag_e motor_status;      // 电机状态标志
} Stepper_motor_measure_t;

/*---------------------------------电机接收结构体声明-----------------------------------*/
extern Stepper_motor_measure_t pitch_motor,yaw_motor;   // 两个步进电机接收信息
/*---------------------------------电机接收结构体声明-----------------------------------*/

/*---------------------------------电机状态信息接收处理函数声明-----------------------------------*/
/**
 * @brief 获取步进电机系统状态
 * @param ptr 指向步进电机测量数据结构的指针
 * @param rx_message 接收到的消息指针
 */
extern void get_stepper_system_status_measure(Stepper_motor_measure_t *ptr, const uint8_t *rx_message);
/*---------------------------------电机状态信息接收处理函数声明-----------------------------------*/

/*---------------------------------电机状态信息指针-----------------------------------*/

Stepper_motor_measure_t* get_yaw_motor_info();
Stepper_motor_measure_t* get_pitch_motor_info();
/*---------------------------------电机状态信息指针-----------------------------------*/


#endif // STEPPER_MOTOR_H
