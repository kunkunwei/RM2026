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

typedef struct 
{
  /* data */
  uint8_t GameState;
  uint8_t Hp_Percentage;
  uint8_t Shoot_Heat_Percentage;
  uint16_t Enerage_buffer;
}refree_info_t;

/**
 * @brief CAN接收回调函数类型
 * @note  BSP层不知道具体的电机类型，通过注册回调将数据上传给Device层解析
 */
typedef void (*BSP_CAN_RxCallback_t)(uint32_t stdId, uint8_t data[8]);

// extern lk9025_motor_measure_t motor_right, motor_left;
// extern dm8009_motor_measure_t motor_joint[4];

// extern CAN_TxFrameTypeDef JointTxFrame[4],ChassisTxFrame,ContorlTxFrame;
// extern CAN_TxFrameTypeDef  RMD_L9025_Left_TxFrame,RMD_L9025_Right_TxFrame,RMD_L9025_ALL_TxFrame;
extern CAN_TxFrameTypeDef ShootTxFrame[3];
extern CAN_TxFrameTypeDef Shoot_Pull_TxFrame;
extern CAN_TxFrameTypeDef ALLShootTxFrame;
extern CAN_TxFrameTypeDef HIGH_CBROAD_FRAME;
extern CAN_TxFrameTypeDef yaokong_CBROAD_FRAME;

extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;
extern CAN_TxFrameTypeDef Chassis_Ctrl_FRAME_1;
extern CAN_TxFrameTypeDef Chassis_Ctrl_FRAME_2;

extern float x;
extern float y;
extern uint16_t hp;
extern uint16_t shoot1_power;
/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);
/**
  * @brief  Register CAN1 receive callback (called in CAN1 Rx FIFO0 ISR).
  */
void BSP_CAN1_RegisterRxCallback(BSP_CAN_RxCallback_t cb);
/**
  * @brief  Register CAN2 receive callback (called in CAN2 Rx FIFO0 ISR).
  */
void BSP_CAN2_RegisterRxCallback(BSP_CAN_RxCallback_t cb);
/**
  * @brief  get
  */
extern refree_info_t refree_info;

#endif //BSP_CAN_H

