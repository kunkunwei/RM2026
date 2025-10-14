//
// Created by kun on 25-7-9.
//

#include "vofa.h"
#include <stdio.h>
#include <string.h>

#include "INS_Task.h"

/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
extern INS_Info_Typedef INS_Info;
// extern dm8009_motor_measure_t motor_joint[4];
Vofa_Frame_t vofa_tx;

// HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,dm8009_motor_measure_t motor_joint[], chassis_move_t* chassis)
// {
//
//
//     Vofa_Frame_t frame={
//
//     .data = {
//         INS_Info.yaw_angle,
//         INS_Info.pit_angle,
//         // INS_Info.rol_angle,
//         chassis->chassis_pitch,
//         chassis->right_leg.leg_angle,
//         chassis->right_leg.leg_length,
//         chassis->left_leg.leg_angle,
//         chassis->left_leg.leg_length,
//         chassis->right_leg.front_joint.tor_set,
//         chassis->right_leg.back_joint.tor_set,
//         chassis->left_leg.front_joint.tor_set,
//         chassis->left_leg.back_joint.tor_set,
//         motor_joint[0].pos,
//         motor_joint[1].pos,
//         motor_joint[2].pos,
//         motor_joint[3].pos,
//     }, // 1初始化数据数组
//         .tail = VOFA_TAIL // 设置JustFloat协议尾部
//     };
//     HAL_StatusTypeDef status;
//
//
//     // 发送整个帧（避免逐字节发送，提高效率）
//     status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
//     return status;
// }
void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 根据需要可增大
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}


