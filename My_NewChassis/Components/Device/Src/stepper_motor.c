//
// Created by kun on 25-7-27.
//

#include "stepper_motor.h"

StepperMotor_Info_t StepperMotor[2]; // 有两个步进电机

void StepperMotor_Init(StepperMotor_Info_t *motor, uint8_t frame_id) {
    motor->frame_id = frame_id;
    motor->enabled = false;
    motor->position = 0;
    motor->pulse_count = 0;
    motor->encoder = 0;
    motor->angle=0;
    motor->position_error = 0;
    motor->stall_flag = 0;
    motor->step_angle = 1.8f; // 1.8度
    motor->subdiv = 16;       // 16细分
}


// void StepperMotor_Update(StepperMotor_Info_t *motor, uint8_t *rx_data, uint8_t len) {
//     if (!motor || !rx_data) return;
//
//     // 假设协议：首字节为命令码
//     uint8_t cmd = rx_data[0];
//
//     switch (cmd) {
//     case 0x30: // 读取编码器
//         if (len >= 3) {
//             motor->encoder = (int16_t)((rx_data[1] << 8) | rx_data[2]);
//         }
//         break;
//     case 0x33: // 读取脉冲计数
//         if (len >= 4) {
//             motor->pulse_count = (int32_t)((rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3]);
//         }
//         break;
//     case 0x36: // 读取位置
//         if (len >= 5) {
//             motor->position = (int32_t)((rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8) | rx_data[4]);
//         }
//         break;
//     case 0x39: // 读取位置误差
//         if (len >= 3) {
//             motor->position_error = (int16_t)((rx_data[1] << 8) | rx_data[2]);
//         }
//         break;
//     case 0x3A: // 读取使能状态
//         if (len >= 2) {
//             motor->enabled = rx_data[1] ? true : false;
//         }
//         break;
//     case 0x3E: // 读取堵转标志
//         if (len >= 2) {
//             motor->stall_flag = rx_data[1];
//         }
//         break;
//     default:
//         // 其他命令可扩展
//         break;
//     }
// }