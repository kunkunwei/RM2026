// #include "can.h"
// #include "bsp_can.h" // Add this include
#include "main.h"
#include "can_task.h"
#include "Chassis_Task.h"
#include "usart.h"
#include "User_Task.h"
#include "vofa.h"

// #define DEBUG
// #define LEG_DEBUG

/* CAN 管理器配置 */
// #define JOINT_SEND_INTERVAL_MS  2   // 关节电机发送间隔: 2ms (高响应)
// #define WHEEL_SEND_INTERVAL_MS  8  // 轮子电机发送间隔: 10ms (低要求)



/* CAN Manager Instance */
CAN_TxManager_t can_manager;

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID,float Postion, float Velocity, float KP, float KD, float Torque);
static void LK9025_Motor_CAN_Send(int16_t right, int16_t left);
static void Damiao_Motor_Enable(uint8_t Motor_ID);




const chassis_move_t* local_chassis ;



#define LEG_DEBUG
void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
    local_chassis = get_chassis_control_point();
  osDelay(1000);
    TickType_t systick = 0;
    Damiao_Motor_Enable(2);
    osDelay(5);
    Damiao_Motor_Enable(3);
    osDelay(5);
  Damiao_Motor_Enable(0);
  osDelay(5);
  Damiao_Motor_Enable(1);
  osDelay(5);
  for(;;)
  {
      CAN_Process_Data(); // Process CAN Rx Buffer
      systick = osKernelSysTick();


    // 关节电机控制
        #ifndef LEG_DEBUG
          Damiao_Motor_CAN_Send(3,0,0,0,local_chassis->right_leg.mit_kd,local_chassis->right_leg.front_joint.tor_set);
          Damiao_Motor_CAN_Send(2,0,0,0,local_chassis->right_leg.mit_kd,local_chassis->right_leg.back_joint.tor_set);
          Damiao_Motor_CAN_Send(0,0,0,0,local_chassis->left_leg.mit_kd,local_chassis->left_leg.front_joint.tor_set);
          Damiao_Motor_CAN_Send(1,0,0,0,local_chassis->left_leg.mit_kd,local_chassis->left_leg.back_joint.tor_set);
        #else
          Damiao_Motor_CAN_Send(0,0,0,0,0,0);
          Damiao_Motor_CAN_Send(1,0,0,0,0,0);
  	// osDelay(1);
          Damiao_Motor_CAN_Send(2,0,0,0,0,0);
          Damiao_Motor_CAN_Send(3,0,0,0,0,0);
        #endif
  	// osDelay(0);
  	// CAN_Process_Data();
    // 轮子电机控制
        #ifndef LEG_DEBUG
          LK9025_Motor_CAN_Send(local_chassis-> right_leg.wheel_motor.give_current,local_chassis->left_leg.wheel_motor.give_current);
        #else
          LK9025_Motor_CAN_Send(0,0);
        #endif
      osDelayUntil(&systick, 1);
  }
  /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID,float Postion, float Velocity, float KP, float KD, float Torque){
    static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
    //uint8_t Motor_ID = ID-1;
    Postion_Tmp  =  float_to_uint(Postion,-12.5f,12.5,16) ;
    Velocity_Tmp =  float_to_uint(Velocity,-45,45,12);
    KP_Tmp = float_to_uint(KP,0,500,12);
    KD_Tmp = float_to_uint(KD,0,5,12);
    Torque_Tmp = float_to_uint(Torque,-20,20,12);
	JointTxFrame[Motor_ID].Data[0] = (uint8_t)(Postion_Tmp>>8);
	JointTxFrame[Motor_ID].Data[1] = (uint8_t)(Postion_Tmp);
	JointTxFrame[Motor_ID].Data[2] = (uint8_t)(Velocity_Tmp>>4);
	JointTxFrame[Motor_ID].Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	JointTxFrame[Motor_ID].Data[4] = (uint8_t)(KP_Tmp);
	JointTxFrame[Motor_ID].Data[5] = (uint8_t)(KD_Tmp>>4);
	JointTxFrame[Motor_ID].Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	JointTxFrame[Motor_ID].Data[7] = (uint8_t)(Torque_Tmp);
	// uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(JointTxFrame[Motor_ID].hcan);

	// // 统计CAN2的邮箱状态
	// static uint32_t can2_tx_count = 0;
	// static uint32_t can2_tx_full = 0;
	// if (JointTxFrame[Motor_ID].hcan->Instance == CAN2) {
	// 	can2_tx_count++;
	// 	if (free_level == 0) {
	// 		can2_tx_full++;
	// 	}
	//
	// 	// 每1000次打印
	// 	if (can2_tx_count % 1000 == 0) {
	// 		uart_printf(&huart1,"[CAN2 TX] Total: %lu, Full: %lu (%.1f%%)\n",
	// 			   can2_tx_count, can2_tx_full,
	// 			   100.0f * can2_tx_full / can2_tx_count);
	// 	}
	// }
   USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
}
static void Damiao_Motor_Enable(uint8_t Motor_ID){
    //uint8_t Motor_ID = ID-1;

	JointTxFrame[Motor_ID].Data[0] = 0xFF;
	JointTxFrame[Motor_ID].Data[1] = 0xFF;
	JointTxFrame[Motor_ID].Data[2] = 0xFF;
	JointTxFrame[Motor_ID].Data[3] = 0xFF;
	JointTxFrame[Motor_ID].Data[4] = 0xFF;
	JointTxFrame[Motor_ID].Data[5] = 0xFF;
	JointTxFrame[Motor_ID].Data[6] = 0xFF;
	JointTxFrame[Motor_ID].Data[7] = 0xFC;

  USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
  // CAN_Manager_Add(&JointTxFrame[Motor_ID]);
}
static void LK9025_Motor_CAN_Send(int16_t right, int16_t left){

  RMD_L9025_ALL_TxFrame.Data[0] = right;
  RMD_L9025_ALL_TxFrame.Data[1] = (right >> 8);
  RMD_L9025_ALL_TxFrame.Data[2] = left;
  RMD_L9025_ALL_TxFrame.Data[3] = (left >> 8);
  RMD_L9025_ALL_TxFrame.Data[4] = 0;
  RMD_L9025_ALL_TxFrame.Data[5] = 0;
  RMD_L9025_ALL_TxFrame.Data[6] = 0;
  RMD_L9025_ALL_TxFrame.Data[7] = 0;

  USER_CAN_TxMessage(&RMD_L9025_ALL_TxFrame);
  // CAN_Manager_Add(&RMD_L9025_ALL_TxFrame);
}
/**
 * @brief 检查CAN发送邮箱是否有空闲空间
 * @return 1: 有空闲邮箱可发送, 0: 邮箱已满
 */
/*
static inline uint8_t CAN_Mailbox_Available(void)
{
	extern CAN_HandleTypeDef hcan1;
	extern CAN_HandleTypeDef hcan2;

	// 检查CAN1和CAN2的发送邮箱，任意一个有空闲即可
	// HAL_CAN_GetTxMailboxesFreeLevel 返回空闲邮箱数量 (0-3)
	uint32_t can1_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	// uint32_t can2_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);

	// 只要有至少1个邮箱空闲，就可以发送
	// return (can1_free > 0 || can2_free > 0);
	return can1_free > 0;
}
*/
/* CAN Manager Implementations */

