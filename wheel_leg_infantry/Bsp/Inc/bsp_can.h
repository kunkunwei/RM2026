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
#include "mymotor.h"
/* Exported types ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_RxHeaderTypeDef header;
    uint8_t 			Data[8];
} CAN_RxFrameTypeDef;

typedef struct {
		CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef header;
    uint8_t				Data[8];
}CAN_TxFrameTypeDef;


// extern lk9025_motor_measure_t motor_right, motor_left;
// extern dm8009_motor_measure_t motor_joint[4];

extern CAN_TxFrameTypeDef JointTxFrame[4],ChassisTxFrame,ContorlTxFrame;
extern CAN_TxFrameTypeDef  RMD_L9025_Left_TxFrame,RMD_L9025_Right_TxFrame,RMD_L9025_ALL_TxFrame;
  extern CAN_TxFrameTypeDef Chassis_Feeback_TxFrame;
/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);
/**
  * @brief  get
  */
// //返回右驱动轮电机变量地址，通过指针方式获取原始数据
// extern const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
// //返回左驱动轮电机变量地址，通过指针方式获取原始数据
// extern const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
// //返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
// extern const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);
#endif //BSP_CAN_H

