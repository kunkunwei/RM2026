//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
// #include "bmi088.h"

#include "stm32f4xx_hal.h" // 根据具体STM32型号调整
#include <stdarg.h>



#include "Gimbal_task.h"
#include "ist8310.h"
#include "pc_uart_ctrl.h"
#include "sbus_remote.h"
// #include "main.h"
#include "Chassis_Task.h"

#define VOFA_CHANNELS 9    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
/* JustFloat协议发送遥控器通道值 */
HAL_StatusTypeDef Vofa_Send_PC_Ctrl_Info(UART_HandleTypeDef *huart, const PC_Ctrl_Info_t *pc_ctrl_info);
HAL_StatusTypeDef Vofa_Send_shoot_Info(UART_HandleTypeDef *huart, const gimbal_t *gimbal);
HAL_StatusTypeDef Vofa_Send_chassis_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis);
extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
