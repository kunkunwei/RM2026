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
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

#define CAN_QUEUE_SIZE 32
#define CAN_MAX_RETRIES 2
  /* CAN优先级定义 */
  typedef enum {
    CAN_PRIORITY_LOW = 0,     // 状态查询、配置命令
    CAN_PRIORITY_NORMAL = 1,  // 普通控制指令
    CAN_PRIORITY_HIGH = 2,    // 电机实时控制
    CAN_PRIORITY_CRITICAL = 3 // 紧急停止、关键指令
} CAN_Priority_e;

  /* CAN帧类型定义 */
  typedef enum {
    CAN_TYPE_DJI_MOTOR = 0,
    CAN_TYPE_STEPPER_CTRL,
    CAN_TYPE_STEPPER_QUERY,
    CAN_TYPE_OTHER
} CAN_Type_e;

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



  /* CAN管理帧结构（包含原有结构体） */
  typedef struct {
    CAN_TxFrameTypeDef base_frame;  // 包含原有的帧结构
    CAN_Priority_e priority;         // 优先级
    CAN_Type_e type;                 // 帧类型
    uint32_t timestamp;              // 时间戳
    uint8_t retry_count;             // 重试次数
    bool need_response;              // 是否需要响应
  } CAN_Managed_Frame_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Configures the CAN Filter.
  */
extern void BSP_CAN_Init(void);
/**
  * @brief  USER function to transmit the Specifies message.
  */
extern void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader);
  /* CAN管理器函数（简洁版本） */
  bool CAN_Send_Frame(CAN_TxFrameTypeDef *frame);
  bool CAN_Send_Frame_Priority(CAN_TxFrameTypeDef *frame, CAN_Priority_e priority);

  /* CAN管理器函数（复杂版本） */
  bool CAN_Send_Frame_Advanced(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc,
                              CAN_Priority_e priority, CAN_Type_e type, bool need_response);

  /* 队列管理函数 */
  void CAN_Process_Send_Queue(void);
  uint32_t CAN_Get_Queue_Size(void);
  void CAN_Clear_Queue(void);
#endif //BSP_CAN_H

