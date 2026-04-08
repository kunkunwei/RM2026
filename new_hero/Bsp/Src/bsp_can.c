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
#include "main.h"

/* Private variables ---------------------------------------------------------*/
/**
 * @brief CAN接收回调函数存储，Device层通过注册接口设置
 */
static BSP_CAN_RxCallback_t s_can1_rx_cb = NULL;
static BSP_CAN_RxCallback_t s_can2_rx_cb = NULL;

/**
 * @brief 注册 CAN1 接收回调函数
 */
void BSP_CAN1_RegisterRxCallback(BSP_CAN_RxCallback_t cb)
{
  s_can1_rx_cb = cb;
}

/**
 * @brief 注册 CAN2 接收回调函数
 */
void BSP_CAN2_RegisterRxCallback(BSP_CAN_RxCallback_t cb)
{
  s_can2_rx_cb = cb;
}

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
CAN_TxFrameTypeDef ALLFricTxFrame = {
    .hcan = &hcan2,
    .header.StdId = CAN2_CMD_ALL_ID,
    .header.IDE = CAN_ID_STD,
    .header.RTR = CAN_RTR_DATA,
    .header.DLC = 8,
};
CAN_TxFrameTypeDef Shoot_Pull_TxFrame = {
    .hcan = &hcan1,
    .header.StdId = CAN1_CMD_PULL_ID,
    .header.IDE = CAN_ID_STD,
    .header.RTR = CAN_RTR_DATA,
    .header.DLC = 8,
};
CAN_TxFrameTypeDef YAW_MOTOR_FRAME = {
    .hcan = &hcan1,
    .header.StdId = CAN1_YAW_MOTOR_ID,
    .header.IDE = CAN_ID_STD,
    .header.RTR = CAN_RTR_DATA,
    .header.DLC = 8,
};
CAN_TxFrameTypeDef PITCH_MOTOR_FRAME = {
    .hcan = &hcan2,
    .header.StdId = CAN2_PITCH_MOTOR_ID,
    .header.IDE = CAN_ID_STD,
    .header.RTR = CAN_RTR_DATA,
    .header.DLC = 8,
};
CAN_TxFrameTypeDef ChassisTxFrame = {
  .hcan = &hcan1,
  .header.StdId = CAN1_CMD_ALL_ID,
  .header.IDE = CAN_ID_STD,
  .header.RTR = CAN_RTR_DATA,
  .header.DLC = 8,
};
/**
 * @brief  Configures the CAN Filter.
 * @param  None
 * @retval None
 */
void BSP_CAN_Init(void)
{
  CAN_FilterTypeDef CAN_FilterConfig = {0};

  /* Update the CAN1 filter Conifguration */
  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_FilterConfig.FilterIdHigh = 0x0000;
  CAN_FilterConfig.FilterIdLow = 0x0000;
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;
  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.SlaveStartFilterBank = 0;

  /* configures the CAN1 filter */
  if (HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the CAN1 module. */
  HAL_CAN_Start(&hcan1);

  /* Enable CAN1 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  /* Update the CAN2 filter Conifguration */
  CAN_FilterConfig.FilterBank = 14;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN2 filter */
  if (HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the CAN2 module. */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{

  static uint32_t TxMailbox = 0;
  HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &TxMailbox);
}
/**
 * @brief  USER function to converting the CAN1 received message.
 * @param  Instance: pointer to the CAN Register base address
 * @param  StdId: Specifies the standard identifier.
 * @param  data: array that contains the received massage.
 * @retval None
 */

static void CAN1_RxFifo0RxHandler(uint32_t *StdId, uint8_t data[8])
{
  if (s_can1_rx_cb != NULL)
  {
    s_can1_rx_cb(*StdId, data);
  }
}

//------------------------------------------------------------------------------

/**
 * @brief  USER function to converting the CAN2 received message.
 * @param  Instance: pointer to the CAN Register base address
 * @param  StdId: Specifies the standard identifier.
 * @param  data: array that contains the received massage.
 * @retval None
 */
static void CAN2_RxFifo0RxHandler(uint32_t *StdId, uint8_t data[8])
{
  if (s_can2_rx_cb != NULL)
  {
    s_can2_rx_cb(*StdId, data);
  }
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

  /* judge the instance of receive frame data */
  if (hcan->Instance == CAN1)
  {
    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId, USER_CAN_RxFrameData);
  }
  else if (hcan->Instance == CAN2)
  {
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId, USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
