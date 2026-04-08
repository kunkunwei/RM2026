//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
// #include "bmi088.h"

#include "stm32f4xx_hal.h" // 根据具体STM32型号调整
#include <stdarg.h>


#include "ctl_chassis.h"
#include "Gimbal_task.h"
#include "ist8310.h"
#include "pc_uart_ctrl.h"
#include "sbus_remote.h"
// #include "main.h"


#define VOFA_CHANNELS 5    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
/* JustFloat协议发送遥控器通道值 */
HAL_StatusTypeDef Vofa_Send_shoot_Info(UART_HandleTypeDef *huart, const gimbal_t *gimbal);
HAL_StatusTypeDef Vofa_Send_gimbal_RC(UART_HandleTypeDef *huart, const chassis_feedback_frame_t *fb );
HAL_StatusTypeDef Vofa_Send_PC_Ctrl_Info(UART_HandleTypeDef *huart, const PC_Ctrl_Info_t *pc_ctrl_info);
// HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,dm8009_motor_measure_t motor_joint[], chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_INS(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,ist8310_real_data_t ist8310_Info);
HAL_StatusTypeDef Vofa_Send_RC(UART_HandleTypeDef *huart, SBUS_Remote_Info_Typedef* sbus_remote);
HAL_StatusTypeDef Vofa_Send_Gimbal(UART_HandleTypeDef *huart, gimbal_t gimbal);
HAL_StatusTypeDef Vofa_Send_Motorr(UART_HandleTypeDef *huart,const gimbal_t* gimbal);
// HAL_StatusTypeDef Vofa_Send_Q(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,Quaternion_Info_Typedef *Quaternion_Info);
// HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Gimbal_Yaw(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Gimbal_Pitch(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, chassis_move_t *chassis);
extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
