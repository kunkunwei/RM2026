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
HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, BMI088_Info_Typedef* BMI088_Info)
{

    // 检查串口状态
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }

    Vofa_Frame_t frame={
    .data = {
        BMI088_Info->accel[0],
        BMI088_Info->accel[1],
        BMI088_Info->accel[2],
        BMI088_Info->gyro[0],
        BMI088_Info->gyro[1],
        BMI088_Info->gyro[2],
        BMI088_Info->temperature,
       0
    }, // 初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;
    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
// HAL_StatusTypeDef Vofa_Send(UART_HandleTypeDef *huart,float* mag)
// {
//
//     // 检查串口状态
//     if (huart->gState != HAL_UART_STATE_READY) {
//         return HAL_BUSY;
//     }
//
//     Vofa_Frame_t frame={
//         .data = {
//             mag[0],
//             mag[1],
//             mag[2],
//         }, // 初始化数据数组
//             .tail = VOFA_TAIL // 设置JustFloat协议尾部
//         };
//     HAL_StatusTypeDef status;
//     // 发送整个帧（避免逐字节发送，提高效率）
//     status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
//     return status;
// }
// HAL_StatusTypeDef Vofa_Send(UART_HandleTypeDef *huart,INS_Info_Typedef* INS)
// {
//
//     // 检查串口状态
//     if (huart->gState != HAL_UART_STATE_READY) {
//         return HAL_BUSY;
//     }
//
//     Vofa_Frame_t frame={
//         .data = {
//            INS->accel[0],
//             INS->accel[1],
//             INS->accel[2],
//             INS->gyro[0],
//             INS->gyro[1],
//             INS->gyro[2],
//         }, // 初始化数据数组
//             .tail = VOFA_TAIL // 设置JustFloat协议尾部
//         };
//     HAL_StatusTypeDef status;
//     // 发送整个帧（避免逐字节发送，提高效率）
//     status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
//     return status;
// }
