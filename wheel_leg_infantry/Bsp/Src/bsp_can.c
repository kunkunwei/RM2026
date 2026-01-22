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
#include "User_Task.h"


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
		.hcan = &hcan1,
		.header.StdId=CAN_dm8009_M1_ID,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
	},
	[1]={
		.hcan = &hcan1,
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
CAN_TxFrameTypeDef Chassis_Feeback_TxFrame ={
    .hcan = &hcan2,
		.header.StdId=0x60,
		.header.IDE=CAN_ID_STD,
		.header.RTR=CAN_RTR_DATA,
		.header.DLC=8,
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
/**
  * @brief  USER function to converting the CAN1 received message.
	* @param  Instance: pointer to the CAN Register base address
	* @param  StdId: Specifies the standard identifier.
	* @param  data: array that contains the received massage.
  * @retval None
  */

static void CAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t data[8])
{
	// float time_now = DWT_GetTimeline_ms();
	// if (*StdId==CAN1_Chassis_ID_1)
	// {
	// 	chassis_parse_control_frame1(&gimbal_chassis_comm.gimbal_cmd, data);
	// 	gimbal_chassis_comm.last_frame1_time=time_now;
	// 	gimbal_chassis_comm.frame1_received =1;
	// }
	// else if (*StdId==CAN1_Chassis_ID_2)
	// {
	// 	chassis_parse_control_frame2(&gimbal_chassis_comm.gimbal_cmd, data);
	// 	gimbal_chassis_comm.last_frame2_time=time_now;
	// 	gimbal_chassis_comm.frame2_received =1;
	// }
	// else
		if(*StdId == 0x141){
	  //RMD_Motor_Info_Update(StdId,data,&RMD_Motor[Right_Wheel]);
    get_lk9025_motor_measure(&motor_right, data);
	}
  else if(*StdId == 0x142){
    get_lk9025_motor_measure(&motor_left, data);
  }
  else{
  	switch(*StdId)
    {
      case CAN_dm8009_M1_ID:
      case CAN_dm8009_M2_ID:
      case CAN_dm8009_M3_ID:
      case CAN_dm8009_M4_ID:  
        {
          // static uint8_t i = 0;
          // //处理电机ID号
          // i = data[0]&0x0F -1 ;
          // //处理电机数据宏函数
          get_dm8009_motor_measure(&motor_joint[*StdId -1 ], data);
          //printf("%.2f\r\n",motor_joint[*StdId -1 ].pos);
          //记录时间
          //detect_hook(CHASSIS_MOTOR1_TOE + i);
          //输出位置修正
          if( *StdId == CAN_dm8009_M1_ID ){
              motor_joint[0].pos += 3.0556903f; // 正确的零点补偿值
          	// motor_joint[0].pos += 3.141593f;

          }
          else if( *StdId== CAN_dm8009_M2_ID){
              motor_joint[1].pos += 0.3876503f; // 正确的零点补偿值
          	// motor_joint[1].pos += 3.141593f/2;


          }
          else if(*StdId == CAN_dm8009_M3_ID){
              motor_joint[2].pos += 3.057657f; // 正确的零点补偿值
          	 // motor_joint[3].pos += 3.141593/2;
          	motor_joint[2].pos = 3.141593f - motor_joint[2].pos;
          }
          else if(*StdId == CAN_dm8009_M4_ID)
          {
              // motor_joint[3].pos += 2.11726f; // 正确的零点补偿值
              motor_joint[3].pos += 2.158088f; // 正确的零点补偿值
          	motor_joint[3].pos = 3.141593/2 - motor_joint[3].pos;
          }
          break;
        }
    }
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
	//DJI_Motor_Info_Update(StdId,data,&DJI_Motor[Yaw]);
//#if (!REMOTE_FRAME_USART_CAN)
//	Remote_Info_Update(StdId,data,&remote_ctrl);
//#endif
	// float time_now = DWT_GetTimeline_ms();
	// if (*StdId==CAN2_YAW_MOTOR_ID)
	// {
	// 	get_dji_motor_measure(&yaw_motor,data);
	// }

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

