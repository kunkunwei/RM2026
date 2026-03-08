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

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "can.h"
#include "remote_control.h"
#include "usart.h"
#include "User_Task.h"
#include "vofa.h"

volatile uint32_t can1_isr_count = 0;
volatile uint32_t can2_isr_count = 0;
volatile uint32_t can1_push_fail = 0;
volatile uint32_t can2_push_fail = 0;
volatile uint32_t can1_last_id = 0;
volatile uint32_t can2_last_id = 0;

extern lk9025_motor_measure_t motor_right, motor_left;
extern dm8009_motor_measure_t motor_joint[4];
extern dji_motor_measure_t yaw_motor;
void get_chassis_ctrl_measure(Chassis_RC_Info_t *rc_info, uint8_t *data);
/* Private variables ---------------------------------------------------------*/
/**
 * @brief the structure that contains the Information of CAN Receive.
 */
CAN_RxHeaderTypeDef USER_CAN_RxInstance;
/**
 * @brief the array that contains the Information of CAN Receive data.
 */
uint8_t USER_CAN_RxFrameData[8];

/**
 * @brief the structure that contains the Information of CAN Transmit.
 */
CAN_TxFrameTypeDef JointTxFrame[4] = {
	[0]={
		.hcan = &hcan2,
		.header.StdId=CAN_dm8009_M1_ID,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[1]={
		.hcan = &hcan2,
		.header.StdId=CAN_dm8009_M2_ID,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
	[2]={
		.hcan = &hcan1,
		.header.StdId=CAN_dm8009_M3_ID,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[3]={
		.hcan = &hcan1,
		.header.StdId=CAN_dm8009_M4_ID,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	}, 
};

// CAN_TxFrameTypeDef dm8009_ALL_TxFrame ={
//     .hcan = &hcan1,
// 		.header.StdId=CAN_CHASSIS_ALL_ID,
// 		.header.IDE=CAN_ID_STD,
// 		.header.RTR=CAN_RTR_DATA,
// 		.header.DLC=8,
// };

CAN_TxFrameTypeDef RMD_L9025_Left_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x142,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};
CAN_TxFrameTypeDef RMD_L9025_Right_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x141,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};

CAN_TxFrameTypeDef RMD_L9025_ALL_TxFrame ={
    .hcan = &hcan1,
		.header.StdId=0x280,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
};

// Ring Buffer Implementation
static CAN_RxFIFO_t can1_rx_fifo;
static CAN_RxFIFO_t can2_rx_fifo;

void CAN_FIFO_Init(void) {
    can1_rx_fifo.head = 0;
    can1_rx_fifo.tail = 0;
    can2_rx_fifo.head = 0;
    can2_rx_fifo.tail = 0;
}

static uint8_t CAN_FIFO_Push(CAN_RxFIFO_t *fifo, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    uint16_t next_head = (fifo->head + 1) % CAN_RX_FIFO_SIZE;
    if (next_head == fifo->tail) {
        return 0; // Buffer full
    }
    fifo->buffer[fifo->head].header = *header;
    memcpy(fifo->buffer[fifo->head].data, data, 8);
    fifo->head = next_head;
    return 1;
}

static uint8_t CAN_FIFO_Pop(CAN_RxFIFO_t *fifo, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    if (fifo->head == fifo->tail) {
        return 0; // Buffer empty
    }
    *header = fifo->buffer[fifo->tail].header;
    memcpy(data, fifo->buffer[fifo->tail].data, 8);
    fifo->tail = (fifo->tail + 1) % CAN_RX_FIFO_SIZE;
    return 1;
}

/**
  * @brief  Configures the CAN Filter.
  * @param  None
  * @retval None
  */
void BSP_CAN_Init(void)
{
    CAN_FilterTypeDef CAN_FilterConfig = {0};

    /* ========== 配置CAN1滤波器 ========== */
    CAN_FilterConfig.FilterActivation = ENABLE;
    CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterBank = 0;              // CAN1使用Bank 0
    CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    CAN_FilterConfig.SlaveStartFilterBank = 14;   //CAN2从Bank 14开始

    /* 配置CAN1滤波器 */
    if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* 启动CAN1 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* ========== 配置CAN2滤波器 ========== */
    CAN_FilterConfig.FilterBank = 14;             // CAN2使用Bank 14
    CAN_FilterConfig.FilterIdHigh = 0x0000;
    CAN_FilterConfig.FilterIdLow = 0x0000;
    CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
    CAN_FilterConfig.FilterMaskIdLow = 0x0000;
    CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    // ⚠️ 注意：不要再设置 SlaveStartFilterBank！

    /* 配置CAN2滤波器 */
    if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK) {
        Error_Handler();
    }

    /* 启动CAN2 */
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* 初始化软件FIFO */
    CAN_FIFO_Init();
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  data: pointer to the CAN transmit data
  * @retval None
  */
//void USER_CAN_TxMessage(CAN_TypeDef *Instance,uint32_t StdId,uint8_t data[8],uint8_t length)
//{
//  static uint32_t TxMailbox = 0;
//	
//  /* Add a message to the first free Tx mailbox and activate the corresponding transmission request. */
//	if(Instance == CAN1)
//	{
//    CAN1_FrameTxInstance.StdId = StdId;
//    CAN1_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan1, &CAN1_FrameTxInstance, data, &TxMailbox);
//	}
//	else if(Instance == CAN2)
//	{
//    CAN2_FrameTxInstance.StdId = StdId;
//    CAN2_FrameTxInstance.DLC = length;
//		HAL_CAN_AddTxMessage(&hcan2, &CAN2_FrameTxInstance, data, &TxMailbox);
//	}
//}
//------------------------------------------------------------------------------
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{
	
	static uint32_t TxMailbox = 0;

//   while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan1 ) == 0 );
	//???????CAN??
	HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &TxMailbox);
}

//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get an CAN frame from the Rx FIFO zone into the message RAM. */
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &USER_CAN_RxInstance, USER_CAN_RxFrameData);

  /* Push to FIFO instead of processing immediately */
  if (hcan->Instance == CAN1) {
      can1_isr_count++;
      can1_last_id = USER_CAN_RxInstance.StdId;
      if (CAN_FIFO_Push(&can1_rx_fifo, &USER_CAN_RxInstance, USER_CAN_RxFrameData) == 0) {
          can1_push_fail++;
      }
  } else if (hcan->Instance == CAN2) {
      can2_isr_count++;
      can2_last_id = USER_CAN_RxInstance.StdId;
      if (CAN_FIFO_Push(&can2_rx_fifo, &USER_CAN_RxInstance, USER_CAN_RxFrameData) == 0) {
          can2_push_fail++;
      }
  }
}

/**
  * @brief  Process CAN data from the FIFO.
	* @param  None
  * @retval None
  */
void CAN_Process_Data(void) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];
    // static uint32_t can1_pop_count = 0;
    // static uint32_t can2_pop_count = 0;
    // static uint32_t can2_m1_count = 0;
    // static uint32_t can2_m2_count = 0;
    // Process CAN1 FIFO
    while (CAN_FIFO_Pop(&can1_rx_fifo, &header, data)) {
        // can1_pop_count++;
        if (header.StdId == CAN_RIGHT_MOTOR_ID) {
            get_lk9025_motor_measure(&motor_right, data);
        } else if (header.StdId == CAN_LEFT_MOTOR_ID) {
            get_lk9025_motor_measure(&motor_left, data);
        } else {
             // Handle DM8009 motors on CAN1 if any (e.g. M3, M4)
             // The original code handled M3/M4 in CAN1 logic (if not M1/M2 which were typically CAN2)
             switch(header.StdId) {
                case CAN_dm8009_M3_ID:
                case CAN_dm8009_M4_ID:
                {
                    get_dm8009_motor_measure(&motor_joint[header.StdId - 1], data);
                    if (header.StdId == CAN_dm8009_M3_ID) {
                        motor_joint[2].pos += 3.007557f;
                        motor_joint[2].pos = 3.141593f - motor_joint[2].pos;
                    } else if (header.StdId == CAN_dm8009_M4_ID) {
                        motor_joint[3].pos += 2.158088f;
                        motor_joint[3].pos = 3.141593f / 2.0f - motor_joint[3].pos;
                    }
                    break;
                }
            }
        }
    }

    // Process CAN2 FIFO
    while (CAN_FIFO_Pop(&can2_rx_fifo, &header, data)) {
        // can2_pop_count++;
        switch(header.StdId) {
        case CAN_dm8009_M1_ID:
            // can2_m1_count++;
            get_dm8009_motor_measure(&motor_joint[0], data);
            motor_joint[0].pos += 3.0556903f;
            break;

        case CAN_dm8009_M2_ID:
            // can2_m2_count++;
            get_dm8009_motor_measure(&motor_joint[1], data);
            motor_joint[1].pos += 0.3876503f;
            break;
        }
    }

    // // 每1000次打印统计
    // static uint32_t debug_count = 0;
    // if (++debug_count >= 1000) {
    //     uart_printf(&huart1,"\n=== CAN Statistics (per 1000 cycles) ===\n");
    //    // uart_printf(&huart1,"CAN1 ISR: %lu, POP: %lu, FAIL: %lu, LastID: 0x%03lX\n",
    //            // can1_isr_count, can1_pop_count, can1_push_fail, can1_last_id);
    //     uart_printf(&huart1,"CAN2 ISR: %lu, POP: %lu, FAIL: %lu, LastID: 0x%03lX\n",
    //            can2_isr_count, can2_pop_count, can2_push_fail, can2_last_id);
    //     uart_printf(&huart1,"CAN2 M1: %lu, M2: %lu\n", can2_m1_count, can2_m2_count);
    //
    //     // 检查软件FIFO占用
    //     uint16_t can1_used = (can1_rx_fifo.head - can1_rx_fifo.tail + CAN_RX_FIFO_SIZE) % CAN_RX_FIFO_SIZE;
    //     uint16_t can2_used = (can2_rx_fifo.head - can2_rx_fifo.tail + CAN_RX_FIFO_SIZE) % CAN_RX_FIFO_SIZE;
    //     uart_printf(&huart1,"SW FIFO - CAN1: %d, CAN2: %d\n", can1_used, can2_used);
    //
    //     // 打印电机数据
    //     // uart_printf(&huart1,"Motor[0](CAN2): pos=%.3f\n",
    //     //        motor_joint[0].pos);
    //     // uart_printf(&huart1,"Motor[1](CAN2): pos=%.3f\n",
    //     //        motor_joint[1].pos);
    //     // uart_printf(&huart1,"Motor[2](CAN1): pos=%.3f\n",
    //            // motor_joint[2].pos);
    //     // uart_printf(&huart1,"Motor[3](CAN1): pos=%.3f\n",
    //            // motor_joint[3].pos);
    //
    //     // 重置计数器
    //     can1_isr_count = 0;
    //     can2_isr_count = 0;
    //     can1_pop_count = 0;
    //     can2_pop_count = 0;
    //     can1_push_fail = 0;
    //     can2_push_fail = 0;
    //     can2_m1_count = 0;
    //     can2_m2_count = 0;
    //     debug_count = 0;
    // }
}
