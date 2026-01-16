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
#include "mymotor.h"
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

CAN_TxFrameTypeDef ALLShootTxFrame ={
    .hcan = &hcan2,
      .header.StdId=CAN2_CMD_ALL_ID,
      .header.IDE=CAN_ID_STD,
      .header.RTR=CAN_RTR_DATA,
      .header.DLC=8,
};


/**
 * @brief Overview:
 * 该文件为 CAN 总线的底层 BSP 层实现，负责 CAN 过滤器配置、消息发送封装、
 * 以及接收到消息的分发（包括 CAN1 的接收处理器）。
 *
 * 接收处理器说明（多帧）：
 * - 本项目中步进电机的多帧上报采用两帧结构：首帧（ExtId = frame_id）携带前 8 字节，
 *   后续帧（ExtId = frame_id + 1）携带剩余字节，且后续帧的 D0 通常放置命令码以便识别。
 * - 系统状态（命令码 0x43）为多帧返回，接收端需将首帧整包（包含 D0..D7）拷贝，
 *   后续每帧从 D1 开始拷贝（D0 为重复的命令码），最终以 [0x43, CAN_CHECK_CODE] 作为结束包。
 */

//------------------------------------------------------------------------------

/**
  * @brief  Configures the CAN Filter.
  * @param  None
  * @retval None
  *
  * @note 初始化 CAN 过滤器、启动 CAN 模块并激活 FIFO0 中断。
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
  if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfig) != HAL_OK)
  {
  Error_Handler();
  }

  /* Start the CAN2 module. */
  HAL_CAN_Start(&hcan2);

  /* Enable CAN2 FIFO0 interrupts */
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
//------------------------------------------------------------------------------

/**
  * @brief  USER function to transmit the Specifies message.
  * @param  TxHeader: 指向封装好的 CAN_TxFrameTypeDef（包含句柄、header、Data）
  * @retval None
  *
  * @note 该接口为发送封装，内部使用 HAL_CAN_AddTxMessage 提交到硬件邮箱。
  */
void USER_CAN_TxMessage(CAN_TxFrameTypeDef *TxHeader)
{

    static uint32_t TxMailbox = 0;

    HAL_CAN_AddTxMessage(TxHeader->hcan, &TxHeader->header, TxHeader->Data, &TxMailbox);
}
/**
  * @brief  USER function to converting the CAN1 received message.
    * @param  StdId: 指向 USER_CAN_RxInstance.StdId 的指针（32 位），
    *         高 16 位为帧 ID，高 8 位或以项目约定解析，低 8 位为包序号（packet index）。
    * @param  data: 接收到的 8 字节数据缓存
  * @retval None
  *
  * @note 该函数负责把收到的 CAN 帧按照应用协议分发给对应的响应处理函数，
  * 包括错误应答、系统状态多帧拼接、以及各命令的即时应答。
  */
static void CAN1_RxFifo0RxHandler(const uint32_t *StdId,uint8_t data[8])
{
    uint16_t frame_id = (*StdId >> 8); // 电机ID
    uint8_t packet_idx = *StdId & 0xFF; // 包序号
    uint8_t len = USER_CAN_RxInstance.DLC;

    // 错误应答（通用错误格式 00 EE ...）
    if (data[0] == 0x00 && data[1] == 0xEE && data[2] == CAN_CHECK_CODE) {
        osSemaphoreRelease(can_cmd_semHandle);
        return;
    }
    // 非目标电机（忽略其他设备或报文）
    if (frame_id != CAN1_YAW_MOTOR_ID && frame_id != CAN1_PITCH_MOTOR_ID) {
        return;
    }
    // 系统状态多帧应答（命令码 0x43）
    if (data[0] == 0x43) {
        // 如果包序号或帧来源变化，重置拼接缓存
        if (packet_idx != expected_packet_idx || frame_id != sys_status_frame_id) {
            sys_status_len = 0;
            sys_status_frame_id = frame_id;
            expected_packet_idx = 0;
        }
        // 防止缓存溢出
        if (sys_status_len + len > sizeof(sys_status_buffer)) {
            sys_status_len = 0;
            expected_packet_idx = 0;
            return;
        }
        if (packet_idx == 0) {
            // 首帧：拷贝完整 8 字节（包含命令码在 D0）
            memcpy(&sys_status_buffer[sys_status_len], data, len);
            sys_status_len += len;
        } else {
            // 后续帧：协议约定 D0 为重复的命令码，因此从 D1 开始拷贝实际剩余数据
            memcpy(&sys_status_buffer[sys_status_len], &data[1], len - 1);
            sys_status_len += (len - 1);
        }
        expected_packet_idx++;
        // 最后一包标志：协议使用 [0x43, CAN_CHECK_CODE] 作为结束包
        if (len == 2 && data[1] == CAN_CHECK_CODE) {
            StepperCAN_ReadSystemStatus_Response(frame_id, sys_status_buffer, sys_status_len);
            sys_status_len = 0;
            expected_packet_idx = 0;
        }
        return;
    }
    // 普通命令应答，根据命令码分发到对应处理函数
    switch (data[0]) {
        case 0xF3:
            StepperCAN_Enable_Response(data, len);
            break;
        case 0xF6:
            StepperCAN_SetSpeed_Response(data, len);
            break;
        case 0xFD:
            StepperCAN_SetPositionAngle_Response(data, len);
            break;
        case 0xFE:
            StepperCAN_EmergencyStop_Response(data, len);
            break;
        case 0x93:
            StepperCAN_SetSingleZero_Response(data, len);
            break;
        case 0x9A:
            StepperCAN_TriggerHome_Response(data, len);
            break;
        default:
            // 其他命令：暂不处理
            break;
    }
 }

//------------------------------------------------------------------------------

/**
  * @brief  USER function to converting the CAN2 received message.
    * @param  StdId: 指向 USER_CAN_RxInstance.StdId 的指针
    * @param  data: 接收到的 8 字节数据缓存
  * @retval None
  */
static void CAN2_RxFifo0RxHandler(const uint32_t *StdId,uint8_t data[8])
{

      switch(*StdId)
    {
          case CAN2_SHOOT_MOTOR_LEFT_ID
                :
                get_dji_motor_measure(&shoot_motor_left,data);
                break;
            case CAN2_SHOOT_MOTOR_RIGHT_ID:
                get_dji_motor_measure(&shoot_motor_right,data);
                break;
            case CAN2_SHOOT_PULL_MOTOR_ID:
                get_dji_motor_measure(&pull_motor,data);
        default:
            break;
    }
}
//------------------------------------------------------------------------------

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  *
  * @note HAL 中断回调会在底层触发此函数，我们从 FIFO 中读取一帧并根据 hcan 实例分发。
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
