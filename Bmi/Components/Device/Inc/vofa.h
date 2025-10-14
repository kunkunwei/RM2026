//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
// #include "bmi088.h"
#include "bmi088.h"
#include "Chassis_task.h"
// #include "stm32f4xx_hal.h" // 根据具体STM32型号调整
// #include "INS_Task.h"
#define VOFA_CHANNELS 10    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

// 修正版重载宏定义
#define Vofa_Send(huart, ...) \
_Generic((FIRST(__VA_ARGS__, NULL)), \
BMI088_Info_Typedef*: _Generic((SECOND(__VA_ARGS__, NULL, NULL)), \
float*: Vofa_SendIMU, \
default: Vofa_SendBMI088 \
), \
INS_Info_Typedef*: Vofa_SendINS \
)(huart, __VA_ARGS__)

#define FIRST(arg1, ...) arg1
#define SECOND(arg1, arg2, ...) arg2

/* JustFloat协议发送遥控器通道值 */
HAL_StatusTypeDef Vofa_SendChassis(UART_HandleTypeDef *huart, chassis_move_t* chassis_move);
// HAL_StatusTypeDef Vofa_SendBMI088(UART_HandleTypeDef *huart, BMI088_Info_Typedef* BMI088_Info);
// HAL_StatusTypeDef Vofa_SendINS(UART_HandleTypeDef *huart, INS_Info_Typedef* INS_Info);
HAL_StatusTypeDef Vofa_SendIMU(UART_HandleTypeDef *huart,  BMI088_Info_Typedef* BMI088_Info,float *mag);

// extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
