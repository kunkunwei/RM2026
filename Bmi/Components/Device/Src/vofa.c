//
// Created by kun on 25-7-9.
//

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
HAL_StatusTypeDef Vofa_SendBMI088(UART_HandleTypeDef *huart, BMI088_Info_Typedef* BMI088_Info) {
    if (huart->gState != HAL_UART_STATE_READY) return HAL_BUSY;

    Vofa_Frame_t frame = {
        .data = {
            BMI088_Info->accel[0],
            BMI088_Info->accel[1],
            BMI088_Info->accel[2],
            BMI088_Info->gyro[0],
            BMI088_Info->gyro[1],
            BMI088_Info->gyro[2],
            BMI088_Info->temperature,
            0
        },
        .tail = VOFA_TAIL
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}

HAL_StatusTypeDef Vofa_SendINS(UART_HandleTypeDef *huart, INS_Info_Typedef* INS_Info) {
    if (huart->gState != HAL_UART_STATE_READY) return HAL_BUSY;

    Vofa_Frame_t frame = {
        .data = {
            INS_Info->accel[0],
            INS_Info->accel[1],
            INS_Info->accel[2],
            INS_Info->gyro[0],
            INS_Info->gyro[1],
            INS_Info->gyro[2],
            0,
            0
        },
        .tail = VOFA_TAIL
    };

    return HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
}
