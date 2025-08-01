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
#include "cmsis_os.h"
#include "remote_control.h"
#include "main.h"
#include "stepper_can.h"
#include "stepper_motor.h"
#include "stepper_response.h"

extern osSemaphoreId can_cmd_semHandle;
static uint8_t sys_status_buffer[32];
static uint16_t sys_status_len = 0;
static uint16_t sys_status_frame_id = 0;
static uint8_t expected_packet_idx = 0;
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

//
// CAN_TxFrameTypeDef YAW_MOTOR_FRAME ={
//   	.hcan = &hcan1,
// 		.header.StdId=CAN1_YAW_MOTOR_ID,
// 		.header.IDE=CAN_ID_EXT,
// 		.header.RTR=CAN_RTR_DATA,
// 		.header.DLC=8,
// };
// CAN_TxFrameTypeDef PITCH_MOTOR_FRAME ={
//   	.hcan = &hcan1,
// 		.header.StdId=CAN1_PITCH_MOTOR_ID,
// 		.header.IDE=CAN_ID_EXT,
// 		.header.RTR=CAN_RTR_DATA,
// 		.header.DLC=8,
// };

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
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
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
  // if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  // {
  //     Error_Handler();
  // }

  /* Start the CAN2 module. */
  // HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  // HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  data: pointer to the CAN transmit data
  * @retval None
  */

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
//
static void CAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{
    uint16_t frame_id = (*StdId); // 标准帧ID即电机ID
    uint8_t len = USER_CAN_RxInstance.DLC;

    // 错误应答
    if (len >= 3 && data[0] == 0x00 && data[1] == 0xEE && data[2] == CAN_CHECK_CODE) {
        osSemaphoreRelease(can_cmd_semHandle);
        return;
    }
    // 非目标电机
    if (frame_id != CAN1_YAW_MOTOR_ID && frame_id != CAN1_PITCH_MOTOR_ID) {
        return;
    }

    // 反馈数据解析
    switch (data[0]) {
        case 0x30: // 读取编码器值
            if (len == 4 && data[3] == CAN_CHECK_CODE) {
                uint16_t encoder = ((uint16_t)data[1] << 8) | data[2];
                StepperCAN_ReadEncoder_Response(frame_id, encoder);
            }
            break;
        case 0x33: // 读取输入脉冲数
            if (len == 6 && data[5] == CAN_CHECK_CODE) {
                int32_t pulse = ((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | data[4];
                StepperCAN_ReadPulseCount_Response(frame_id, pulse);
            }
            break;
        case 0x36: // 读取实时位置
            if (len == 6 && data[5] == CAN_CHECK_CODE) {
                int32_t position = ((int32_t)data[1] << 24) | ((int32_t)data[2] << 16) | ((int32_t)data[3] << 8) | data[4];
                StepperCAN_ReadPosition_Response(frame_id, position);
            }
            break;
        case 0x39: // 读取位置误差
            if (len == 4 && data[3] == CAN_CHECK_CODE) {
                int16_t error = ((int16_t)data[1] << 8) | data[2];
                StepperCAN_ReadError_Response(frame_id, error);
            }
            break;
        case 0x3A: // 读取使能状态
            if (len == 3 && data[2] == CAN_CHECK_CODE) {
                uint8_t enable = data[1];
                StepperCAN_ReadEnableStatus_Response(frame_id, enable);
            }
            break;
        case 0x3E: // 读取堵转标志
            if (len == 3 && data[2] == CAN_CHECK_CODE) {
                uint8_t stall = data[1];
                StepperCAN_ReadStallFlag_Response(frame_id, stall);
            }
            break;
        case 0x3F: // 读取自动回零状态
            if (len == 3 && data[2] == CAN_CHECK_CODE) {
                uint8_t home_status = data[1];
                StepperCAN_ReadAutoHomeStatus_Response(frame_id, home_status);
            }
            break;
        case 0x0A: // 设置当前位置为零点应答
            if (len == 3 && data[2] == CAN_CHECK_CODE) {
                StepperCAN_SetZero_Response(frame_id);
            }
            break;
        case 0x0E: // 解除堵转保护应答
            if (len == 4 && data[3] == CAN_CHECK_CODE) {
                StepperCAN_UnlockStallProtection_Response(frame_id);
            }
            break;
        case 0xFD: // 运动命令反馈
            StepperCAN_SetPositionAngle_Response(data, len, frame_id);
            break;
        case 0xF3:
            StepperCAN_Enable_Response(data, len, frame_id);
            break;
        case 0xF6:
            StepperCAN_SetSpeed_Response(data, len, frame_id);
            break;
        case 0xFE:
            StepperCAN_EmergencyStop_Response(data, len, frame_id);
            break;
        case 0x93:
            StepperCAN_SetSingleZero_Response(data, len, frame_id);
            break;
        case 0x9A:
            StepperCAN_TriggerHome_Response(data, len, frame_id);
            break;
        default:
            // 其他命令
            break;
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
static void CAN2_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{
    //printf("CAN2 STDID: %d\r\n",*StdId);
  	switch(*StdId)
    {

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
  if(hcan->Instance == CAN1)
  {
    CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
  else if(hcan->Instance == CAN2)
  {
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
