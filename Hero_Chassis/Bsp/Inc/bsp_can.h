/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : Pay attention to enable the can filter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"

/* CAN发送和接收ID定义 */
// #define CAN1_CMD_ALL_ID      0x200  // 底盘电机控制
// #define CAN2_CMD_ALL_ID      0x1FF  // 云台电机控制
// #define CAN2_SHOOT_ALL_ID    0x2FF  // 发射机构控制

/* 电机ID定义 */
// 底盘电机ID (M3508)
// #define CAN2_WHEEL_MOTRO_RIGHT_FRONT_ID    0x201
// #define CAN2_WHEEL_MOTRO_LEFT_FRONT_ID     0x202
// #define CAN2_WHEEL_MOTRO_LEFT_BACK_ID      0x203
// #define CAN2_WHEEL_MOTRO_RIGHT_BACK_ID     0x204

// 云台电机ID
// #define CAN2_YAW_MOTOR_ID                  0x205
// #define CAN2_PITCH_MOTOR_ID                0x206
// #define CAN2_TRIGGER_MOTOR_ID              0x207

/* CAN发送和接收帧结构体定义 */
typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t		Data[8];
} CAN_RxFrameTypeDef;

typedef struct {
    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t		Data[8];
} CAN_TxFrameTypeDef;

/* 发送帧实例声明 */
extern CAN_TxFrameTypeDef ALLShootTxFrame;    // 发射机构发送帧
extern CAN_TxFrameTypeDef ALLChassisTxFrame;  // 底盘发送帧
extern CAN_TxFrameTypeDef GimbalTxFrame[];   // 云台Yaw轴Pitch轴发送帧


/* 函数声明 */
void BSP_CAN_Init(void);
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);



#ifdef __cplusplus
}
#endif

#endif // BSP_CAN_H