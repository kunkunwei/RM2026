/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_can.c
  * @brief          : bsp can functions for new stepper motor
  ******************************************************************************
  */
/* USER CODE END Header */

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

  /* Update the CAN1 filter Configuration for Extended ID */
  CAN_FilterConfig.FilterActivation = ENABLE;
  CAN_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  CAN_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  // 配置扩展帧过滤器 - 接收所有扩展帧
  // FilterIdHigh 和 FilterIdLow 组合成29位扩展ID
  // 设置IDE位为1表示扩展帧
  CAN_FilterConfig.FilterIdHigh = 0x0000;       // 扩展ID高位
  CAN_FilterConfig.FilterIdLow = 0x0001;        // 扩展ID低位 + IDE位(bit2) = 1
  CAN_FilterConfig.FilterMaskIdHigh = 0x0000;   // 掩码高位 - 全0表示不过滤
  CAN_FilterConfig.FilterMaskIdLow = 0x0000;    // 掩码低位 - 只检查IDE位

  CAN_FilterConfig.FilterBank = 0;
  CAN_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  CAN_FilterConfig.SlaveStartFilterBank = 14;

  /* configures the CAN1 filter */
  if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfig) != HAL_OK)
  {	
      Error_Handler();
  }

  /* Start the CAN1 module. */
  if(HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  /* Enable CAN1 FIFO0 interrupts */
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
      Error_Handler();
  }

  // 添加调试信息输出
  // printf("CAN1 Init Complete - Filter configured for Extended Frames\r\n");
  // printf("Listening for Motor IDs: 0x%03X, 0x%03X\r\n", CAN1_YAW_MOTOR_ID, CAN1_PITCH_MOTOR_ID);

  /* Update the CAN2 filter Configuration */
  // CAN_FilterConfig.FilterBank = 14;
  // CAN_FilterConfig.SlaveStartFilterBank = 14;

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
  */
static void CAN1_RxFifo0RxHandler(uint32_t *ExtId, uint8_t data[8])
{
    uint16_t frame_id = (*ExtId) & 0xFFFF; // 提取扩展帧ID
    // uint16_t frame_id = *StdId;
    uint8_t len = USER_CAN_RxInstance.DLC;

    // 错误应答检查
    if (len >= 3 && data[0] == 0x00 && data[1] == 0xEE && data[2] == CAN_CHECK_CODE) {
        return;
    }

    // 检查是否为目标电机（考虑分包的第二帧ID）
    bool is_target_motor = (frame_id == CAN1_YAW_MOTOR_ID || frame_id == CAN1_PITCH_MOTOR_ID ||
                           frame_id == (CAN1_YAW_MOTOR_ID + 1) || frame_id == (CAN1_PITCH_MOTOR_ID + 1));

    if (!is_target_motor) {
        return;
    }

    // 获取基础电机ID（去除分包偏移）
    uint16_t base_motor_id = (frame_id == (CAN1_YAW_MOTOR_ID + 1)) ? CAN1_YAW_MOTOR_ID :
                            (frame_id == (CAN1_PITCH_MOTOR_ID + 1)) ? CAN1_PITCH_MOTOR_ID : frame_id;

    // 反馈数据解析
    switch (data[0]) {
        case 0x36: // 读取实时位置反馈
            if (len == 7 && data[6] == CAN_CHECK_CODE) {
                uint8_t sign = data[1];
                int32_t position = ((int32_t)data[2] << 24) | ((int32_t)data[3] << 16) |
                                  ((int32_t)data[4] << 8) | data[5];
                StepperCAN_ReadPosition_Response(base_motor_id, sign, position);
            }
            break;

        case 0xFD: // 位置控制命令反馈
            StepperCAN_SetPositionAngle_Response(data, len, base_motor_id);
            break;

        case 0xF3: // 使能命令反馈
            StepperCAN_Enable_Response(data, len, base_motor_id);
            break;

        case 0xFE: // 紧急停止反馈
            StepperCAN_EmergencyStop_Response(data, len, base_motor_id);
            break;

        case 0x9A: // 回零命令反馈
            StepperCAN_TriggerHome_Response(data, len, base_motor_id);
            break;

        case 0x9F: // 运动完成反馈
            if (len == 3 && data[2] == CAN_CHECK_CODE) {
                StepperCAN_SetPositionAngle_Response(data, len, base_motor_id);
            }
            break;

        default:
            // 未知命令，忽略
            break;
    }
}

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
        // 使用扩展帧ID
        CAN1_RxFifo0RxHandler(&USER_CAN_RxInstance.ExtId, USER_CAN_RxFrameData);
  }
  else if(hcan->Instance == CAN2)
  {
    CAN2_RxFifo0RxHandler(&USER_CAN_RxInstance.StdId,USER_CAN_RxFrameData);
  }
}
//------------------------------------------------------------------------------
