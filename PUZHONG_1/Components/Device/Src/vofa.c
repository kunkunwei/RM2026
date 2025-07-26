//
// Created by kun on 25-7-9.
//

#include "vofa.h"

#include <string.h>

/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
Vofa_Frame_t vofa_tx;
HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, emm42_motor_measure_t* stepper_motor)
{


    Vofa_Frame_t frame={
    .data = {
        stepper_motor->encoder,
        (float)stepper_motor->pos,
        stepper_motor->angle,
        stepper_motor->angular_velocity,
        stepper_motor->accel,
        stepper_motor->target_pulses,
        stepper_motor->sent_pulses,
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

