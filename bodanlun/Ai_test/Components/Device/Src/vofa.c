//
// Created by kun on 25-7-9.
//
#include "Gimbal_task.h"
#include "vofa.h"
// #include "ibus.h"

#include "stm32f4xx_hal.h" // 根据具体STM32型号调整
#include <string.h>

/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
Vofa_Frame_t vofa_tx;
HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, gimbal_shoot_t* shoot, uint16_t num_channels)
{


    // 检查串口状态
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    Vofa_Frame_t frame;
    HAL_StatusTypeDef status;
    // shoot->pull_motor->rpm
    // 填充数据
    // for (uint16_t i = 0; i < num_channels; i++) {
        // frame.data[i] = rc_data->channels[i] / 10.0f; // 将通道值转换为浮点数
    // }
    frame.data[0]=shoot->pull_motor->rpm;
    frame.data[1]=shoot->pull_motor->real_w;
    frame.data[2]=shoot->pull_motor->speed;
    frame.data[3]=shoot->pull_motor->target_current;
    frame.data[4]=shoot->pull_motor->real_pos;
    frame.data[5]=shoot->pull_motor->pos;
    // frame.data[0] =12;
    // // 填充剩余通道（若num_channels < VOFA_CHANNELS）
    // for (uint16_t i = num_channels; i < VOFA_CHANNELS; i++) {
    //     frame.data[i] = 0.0f; // 用0填充未使用通道
    // }
    // 设置JustFloat协议尾部
    frame.tail[0] = 0x00;
    frame.tail[1] = 0x00;
    frame.tail[2] = 0x80;
    frame.tail[3] = 0x7F;

    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
