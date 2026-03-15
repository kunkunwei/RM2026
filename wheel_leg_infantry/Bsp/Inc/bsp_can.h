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

extern CAN_TxFrameTypeDef JointTxFrame[4];
extern CAN_TxFrameTypeDef  RMD_L9025_Left_TxFrame,RMD_L9025_Right_TxFrame,RMD_L9025_ALL_TxFrame;

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
#define CAN_RX_FIFO_SIZE 128

typedef struct {
    CAN_RxHeaderTypeDef header;
    uint8_t             data[8];
} CAN_RxFrameNode;

typedef struct {
    CAN_RxFrameNode buffer[CAN_RX_FIFO_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} CAN_RxFIFO_t;

void CAN_Process_Data(void);

// extern lk9025_motor_measure_t motor_right, motor_left;

#endif //BSP_CAN_H

