//
// Created by kun on 25-7-9.
//

#include "../Inc/vofa.h"

#include <stdio.h>
#include <string.h>

/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
extern INS_Info_Typedef INS_Info;
// extern control_mode_t control_mode;
// extern gimbal_t gimbal;
Vofa_Frame_t vofa_tx;
HAL_StatusTypeDef Vofa_Send_Gimbal_Yaw(UART_HandleTypeDef *huart, const gimbal_t *gimbal)
{


    Vofa_Frame_t frame={
    .data = {
       gimbal->gimbal_pos.yaw_motor->add_angle,
       gimbal->gimbal_pos.yaw_motor->target_angle,
       gimbal->gimbal_pos.yaw_motor->given_current,
       gimbal->gimbal_pos.yaw_motor->absolute_angle_set,
       gimbal->gimbal_pos.yaw_motor->relative_angle_set,
       gimbal->gimbal_pos.yaw_motor->relative_angle,

       gimbal->gimbal_pos.yaw_motor->motor_measure->pos_int,
       gimbal->gimbal_pos.yaw_motor->motor_measure->pos,
       gimbal->gimbal_mode,
       //  // gimbal->init_manager.init_state,
       //
       INS_Info.yaw_angle,
        gimbal->gimbal_pos.yaw_motor-> absolute_angle,
       //  INS_Info.pit_angle,
       // //
       //  gimbal->gimbal_pos.pitch_motor->add_angle,
       // gimbal->gimbal_pos.pitch_motor->target_angle,
       // gimbal->gimbal_pos.pitch_motor->given_current,
       // gimbal->gimbal_pos.pitch_motor->absolute_angle_set,
       // gimbal->gimbal_pos.pitch_motor->relative_angle_set,
        // gimbal->gimbal_mode
       // gimbal->gimbal_pos.pitch_motor->motor_measure->pos_int,
       // gimbal->gimbal_pos.pitch_motor->motor_measure->pos,
    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Gimbal_Pitch(UART_HandleTypeDef *huart, gimbal_t *gimbal)
{


    Vofa_Frame_t frame={
        .data = {
            // gimbal->gimbal_pos.pitch_motor->add_angle,
            // gimbal->gimbal_pos.pitch_motor->target_angle,
            // gimbal->gimbal_pos.pitch_motor->given_current,
            // gimbal->gimbal_pos.pitch_motor->absolute_angle_set,
            // gimbal->gimbal_pos.pitch_motor->relative_angle_set,
            // // gimbal->gimbal_pos.yaw_motor->motor_measure->total_round,
            // gimbal->gimbal_pos.pitch_motor->motor_measure->pos_int,
            // gimbal->gimbal_pos.pitch_motor->motor_measure->pos,
            //
            //  INS_Info.yaw_angle,
            //  INS_Info.pit_angle,
         }, // 1初始化数据数组
             .tail = VOFA_TAIL // 设置JustFloat协议尾部
         };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, chassis_move_t *chassis)
{


    Vofa_Frame_t frame={
    .data = {
      // chassis->motor_measure[0]->rpm,
      // chassis->motor_measure[1]->rpm,
      // chassis->motor_measure[2]->rpm,
      // chassis->motor_measure[3]->rpm,
      chassis->vel_set.wz,

       // gimbal->gimbal_pos.yaw_motor_measure->deta_pos,
        INS_Info.yaw_angle,
        chassis->chassis_yaw,
        chassis->gimbal_yaw_motor->relative_angle,
        // chassis.

        // INS_Info.pit_angle,
    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 根据需要可增大
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

