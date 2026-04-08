//
// Created by kun on 25-7-9.
//

#include "../Inc/vofa.h"
#include <stdio.h>
#include <string.h>

#include "Gimbal_task.h"
#include "sbus_remote.h"


/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
extern INS_Info_Typedef INS_Info;
extern ist8310_real_data_t ist8310_Info;

Vofa_Frame_t vofa_tx;

HAL_StatusTypeDef Vofa_Send_shoot_Info(UART_HandleTypeDef *huart, const gimbal_t *gimbal)
{


    Vofa_Frame_t frame={

        .data = {
            gimbal->gimbal_shoot.shoot_target_speed,
            gimbal->gimbal_shoot.shoot_speed,
            gimbal->gimbal_shoot.shoot_right_rad,
            gimbal->gimbal_shoot.shoot_left_rad,
            // gimbal->gimbal_shoot.bullet_done,
            gimbal->gimbal_shoot.pull_target_speed,
            gimbal->gimbal_shoot.pull_motor->target_current,
            gimbal->gimbal_shoot.Pull_PID.out,
            // gimbal->gimbal_pos.yaw_motor_measure->pos
            gimbal->gimbal_RC->rc.ch[0],
            gimbal->gimbal_RC->rc.ch[1],
            gimbal->gimbal_RC->rc.ch[2],
            gimbal->gimbal_RC->rc.ch[4],

        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_INS(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,ist8310_real_data_t ist8310_Info)
{


    Vofa_Frame_t frame={

        .data = {
            INS_Info.yaw_angle,
            INS_Info.pit_angle,
            INS_Info.rol_angle,
            // ist8310_Info.raw_mag[0],
            // ist8310_Info.raw_mag[1],
            // ist8310_Info.raw_mag[2],
            ist8310_Info.calibrated_mag[0],
            ist8310_Info.calibrated_mag[1],
            ist8310_Info.calibrated_mag[2],
            // ist8310_Info.mag_max[0],
            // ist8310_Info.mag_max[1],
            // ist8310_Info.mag_max[2],
            // ist8310_Info.mag_min[0],
            // ist8310_Info.mag_min[1],
            // ist8310_Info.mag_min[2],
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Gimbal(UART_HandleTypeDef *huart, gimbal_t gimbal)
{


    Vofa_Frame_t frame={

        .data = {
            // gimbal.gimbal_pos.yaw_absolute_pos,
            gimbal.gimbal_pos.init_pitch_pos,
            gimbal.gimbal_pos.pitch_target_pos,
            gimbal.gimbal_pos.pitch_absolute_pos,
            gimbal.gimbal_mod,

            // gimbal.gimbal_pos.roll_absolute_pos,
            // gimbal.gimbal_pos.add_yaw,
            // gimbal.gimbal_pos.yaw_target_pos,
            // gimbal.gimbal_pos.yaw_absolute_pos,
            // gimbal.gimbal_pos.yaw_relattive_pos,
            // gimbal.gimbal_pos.yaw_motor_measure->pos,
            // gimbal.gimbal_pos.yaw_motor_measure->target_tor,
            // gimbal.gimbal_pos.yaw_motor_measure->tor,
            // gimbal.gimbal_pos.pitch_absolute_pos,
            // gimbal.gimbal_pos.roll_absolute_pos,
            // gimbal.gimbal_pos.pitch_target_pos,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_RC(UART_HandleTypeDef *huart, SBUS_Remote_Info_Typedef* sbus_remote)
{


    Vofa_Frame_t frame={

        .data = {
            sbus_remote->rc.ch[0],
            sbus_remote->rc.ch[1],
            sbus_remote->rc.ch[2],
            sbus_remote->rc.ch[3],
            sbus_remote->rc.ch[4],
            sbus_remote->rc.s[0],
            sbus_remote->rc.s[1],
            sbus_remote->rc.s[2],
            sbus_remote->rc.s[3],
            sbus_remote->rc_lost
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

