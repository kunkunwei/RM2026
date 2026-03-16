//
// Created by kun on 25-7-9.
//

#ifndef VOFA_H
#define VOFA_H

// #include "ibus.h"
// #include "bmi088.h"

#include "stm32f4xx_hal.h" // 根据具体STM32型号调整
#include <stdarg.h>

#include "Chassis_task.h"
#include "ist8310.h"
#include "leg_angular_predictor.h"
#include "slip_detector.h"
// #include "User_Task.h"
// #include "main.h"


#define VOFA_CHANNELS 13    // VOFA+通道数量
#define VOFA_TAIL {0x00, 0x00, 0x80, 0x7F} // JustFloat协议尾部

typedef  struct {
    float data[VOFA_CHANNELS]; // 通道数据（小端序）
    uint8_t tail[4];           // JustFloat协议固定尾部
} Vofa_Frame_t;

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...);
/* JustFloat协议发送遥控器通道值 */
HAL_StatusTypeDef Vofa_Send_PC_Ctrl_Info(UART_HandleTypeDef *huart, const PC_Ctrl_Info_t *pc_ctrl_info);
HAL_StatusTypeDef Vofa_Send_System(UART_HandleTypeDef *huart, const chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_New_Chassis_Data(UART_HandleTypeDef *huart, const chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_joint_angle(UART_HandleTypeDef *huart, const chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_Tor(UART_HandleTypeDef *huart, const chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,dm8009_motor_measure_t motor_joint[], chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_Data(UART_HandleTypeDef *huart, const chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_INS(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,ist8310_real_data_t ist8310_Info);
HAL_StatusTypeDef Vofa_Send_Q(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,Quaternion_Info_Typedef *Quaternion_Info);
HAL_StatusTypeDef Vofa_Send_Balance(UART_HandleTypeDef *huart, chassis_move_t* chassis);
HAL_StatusTypeDef Vofa_Send_Slip(UART_HandleTypeDef *huart, chassis_move_t* chassis,SlipDetector_t *detector);
HAL_StatusTypeDef Vofa_Send_Pred(UART_HandleTypeDef *huart,chassis_move_t* chassis );
HAL_StatusTypeDef Vofa_Send_Theata(UART_HandleTypeDef *huart,chassis_move_t* chassis );
HAL_StatusTypeDef Vofa_Send_Theata_pre(UART_HandleTypeDef *huart,const chassis_move_t* chassis ,const LegPredictor_t *predictor,const SlipDetector_t *detector);
HAL_StatusTypeDef Vofa_Send_Calibrate(UART_HandleTypeDef *huart,const chassis_move_t* chassis);
// HAL_StatusTypeDef Vofa_SendRCChannels(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Gimbal_Yaw(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Gimbal_Pitch(UART_HandleTypeDef *huart, gimbal_t *gimbal);
// HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, chassis_move_t *chassis);
extern Vofa_Frame_t vofa_tx;
#endif //VOFA_H
