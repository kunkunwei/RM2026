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

HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info)
{


    Vofa_Frame_t frame={

    .data = {
        INS_Info.yaw_angle,
        INS_Info.pit_angle,
        // INS_Info.rol_angle,

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
    char buffer[128];
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (len > 0 && len < sizeof(buffer)) {
        // 使用非阻塞发送，需要确保之前的数据已发送完成
        if (huart->gState == HAL_UART_STATE_READY) {
            HAL_UART_Transmit(huart, (uint8_t *)buffer, len, 100);
        }
        // 如果UART忙，可以选择丢弃数据或加入队列
    }
}


