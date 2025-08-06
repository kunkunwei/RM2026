//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
#include "bmi088.h"
#include "stm32f4xx_hal.h" // 根据具体STM32型号调整
#include "INS_Task.h"
#define VOFA_CHANNELS 8    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

#define Vofa_Send(huart, arg) _Generic((arg), \
BMI088_Info_Typedef*: Vofa_SendBMI088, \
INS_Info_Typedef*: Vofa_SendINS \
)(huart, arg)

/* JustFloat协议发送遥控器通道值 */

HAL_StatusTypeDef Vofa_SendBMI088(UART_HandleTypeDef *huart, BMI088_Info_Typedef* BMI088_Info);
HAL_StatusTypeDef Vofa_SendINS(UART_HandleTypeDef *huart, INS_Info_Typedef* INS_Info);

// extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
