//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
#include "stm32f4xx_hal.h" // 根据具体STM32型号调整

#define VOFA_CHANNELS 8    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

// extern gimbal_t gimbal;
/* JustFloat协议发送遥控器通道值 */
HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, gimbal_shoot_t* shoot, uint16_t num_channels);

extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
