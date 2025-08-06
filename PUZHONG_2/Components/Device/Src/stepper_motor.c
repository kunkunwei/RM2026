//
// Created by kun on 25-7-27.
//

#include "stepper_motor.h"

#include <stddef.h>

Stepper_motor_measure_t pitch_motor,yaw_motor; // 两个步进电机



// 解析系统状态参数
void get_stepper_system_status_measure(Stepper_motor_measure_t *ptr, const uint8_t *rx_message)
{
    // rx_message[0] == 0x43
    ptr->param_len    = rx_message[1];
    ptr->param_count  = rx_message[2];
    ptr->bus_voltage  = (rx_message[3] << 8) | rx_message[4];
    ptr->bus_current  = (rx_message[5] << 8) | rx_message[6];
    ptr->encoder_value= (rx_message[7] << 8) | rx_message[8];

    // 电机目标位置（符号+4字节）
    int sign = rx_message[9] ? -1 : 1;
    uint32_t pos = (rx_message[10]<<24) | (rx_message[11]<<16) | (rx_message[12]<<8) | rx_message[13];
    ptr->target_position = sign * (int32_t)pos;

    // 实时转速（符号+2字节）
    sign = rx_message[14] ? -1 : 1;
    uint16_t spd = (rx_message[15]<<8) | rx_message[16];
    ptr->realtime_speed = sign * (int16_t)spd;

    // 实时位置（符号+4字节）
    sign = rx_message[17] ? -1 : 1;
    pos = (rx_message[18]<<24) | (rx_message[19]<<16) | (rx_message[20]<<8) | rx_message[21];
    ptr->realtime_position = sign * (int32_t)pos;

    // 位置误差（符号+4字节）
    sign = rx_message[22] ? -1 : 1;
    pos = (rx_message[23]<<24) | (rx_message[24]<<16) | (rx_message[25]<<8) | rx_message[26];
    ptr->position_error = sign * (int32_t)pos;

    // 状态标志
    ptr->home_status  = rx_message[27];
    ptr->motor_status = rx_message[28];
}

//返回指针
// StepperMotor_Info_t* get_stepper_motor_info(uint8_t index) {
//     if (index < 2) {
//         return &StepperMotor[index];
//     }
//     return NULL; // 超出范围返回NULL
// }
Stepper_motor_measure_t* get_yaw_motor_info() {
    return &yaw_motor;
}
Stepper_motor_measure_t* get_pitch_motor_info() {
    return &pitch_motor;
}