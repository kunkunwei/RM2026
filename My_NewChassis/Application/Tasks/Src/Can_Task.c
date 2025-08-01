#include "main.h"
#include "can_task.h"
#include "Chassis_Task.h"
#include "old_pid.h"
#include "stepper_can.h"
#include "stepper_motor.h"
//#define DEBUG
static void Damiao_Motor_Enable();
static void Damiao_Motor_CAN_Send(float Postion, float Velocity, float KP, float KD, float Torque);
static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left,int16_t current_Shoot_Right,int16_t current_Shoot_Pull);
static void Dji_Motor_Chassis_Can_Send(int16_t current_id_1,int16_t current_id_2,int16_t current_id_3,int16_t current_id_4);
static void Dji_Motor_Pitch_Can_Send(int16_t current_pitch);

extern CAN_TxFrameTypeDef ALLShootTxFrame;
extern CAN_TxFrameTypeDef ALLChassisTxFrame;
extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;

void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
  // extern gimbal_t gimbal;
  // chassis_move_t* can_chassis_move = get_chassis_point();
  osDelay(800);
  // Damiao_Motor_Enable();
  osDelay(10);
  // Damiao_Motor_Enable();
  osDelay(10);
	StepperMotor_Init(&StepperMotor[0],0x01);
	StepperMotor_Init(&StepperMotor[1],0x02);
	StepperCAN_Enable(StepperMotor[0].frame_id,ENABLE);
	StepperCAN_Enable(StepperMotor[1].frame_id,ENABLE);
	// StepperCAN_SetSpeed(0x01,200,1,10);
	// StepperCAN_RelativeMoveAngle( &StepperMotor[0], 200, 0, 10, 90.0f);
  for(;;)
  {
  	StepperCAN_ReadPosition( StepperMotor[0].frame_id);
  	// StepperCAN_RelativeMoveAngle( &StepperMotor[0], 200,  10, remote_ctrl.rc.ch[0]);
  	// if (switch_is_up(remote_ctrl.rc.s[0]))
  	StepperCAN_RelativeMoveAngle( &StepperMotor[0], 200,  10, 90.0f);
  	// StepperCAN_SetSpeed(0x01,200,1,10);
  	// 仅作LED显示
  	// StepperCAN_SetSpeed(0,200,0,0);
  	// StepperCAN_SetSpeed(1,200,0,10);
  	// StepperCAN_RelativeMove(0x01, 200, 0, 10, 1000);
  	// StepperCAN_RelativeMoveAngle( &StepperMotor[0], 200, 0, 10, 90.0f);
  	// tset_can();
  	// StepperMotor_Update(1, USER_CAN_RxFrameData, &RMD_Motor[0]);
  	 // if (remote_ctrl.rc.s[0]== 0)
  	 // {
  	 // 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  	 // 	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  	 // }
  	 // else
  	 // {
  	 // 	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  	 // 	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  	 // }
     // //摩擦轮加拨蛋
     // Dji_Motor_Shoot_Can_Send(gimbal.gimbal_shoot.shoot_motor_left->target_current,\
     //                           gimbal.gimbal_shoot.shoot_motor_right->target_current,\
     //                           gimbal.gimbal_shoot.pull_motor->target_current);
    // 摩擦轮加拨蛋加pitch
     // Dji_Motor_Shoot_Can_Send(0,0,0,0);
    // 四个轮子
     // Dji_Motor_Chassis_Can_Send(chassis_move.chassis_motor[0]->target_current,\
     //                             chassis_move.chassis_motor[1]->target_current,\
     //                             chassis_move.chassis_motor[2]->target_current,\
     //                             chassis_move.chassis_motor[3]->target_current);
    // Dji_Motor_Pitch_Can_Send(gimbal.gimbal_pos.pitch_motor->target_current);
    // Dji_Motor_Pitch_Can_Send(0);//
    //// 单独的达妙YAW
    // Damiao_Motor_Enable();
    // Dji_Motor_Chassis_Can_Send(0,0,0,0);
    // Damiao_Motor_Enable();
    // Damiao_Motor_CAN_Send(0,0,0,0,0);
    // Dji_Motor_Shoot_Can_Send(500,500,500);


    osDelay(1000);
  }
  /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

static void Damiao_Motor_CAN_Send(float Postion, float Velocity, float KP, float KD, float Torque){
  static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;

  Postion_Tmp  =  float_to_uint(Postion,-12.5f,12.5,16) ;
  Velocity_Tmp =  float_to_uint(Velocity,-30,30,12);
  KP_Tmp = float_to_uint(KP,0,500,12);
  KD_Tmp = float_to_uint(KD,0,5,12);
  Torque_Tmp = float_to_uint(Torque,-10,10,12);
	YAW_MOTOR_FRAME.Data[0] = (uint8_t)(Postion_Tmp>>8);
	YAW_MOTOR_FRAME.Data[1] = (uint8_t)(Postion_Tmp);
	YAW_MOTOR_FRAME.Data[2] = (uint8_t)(Velocity_Tmp>>4);
	YAW_MOTOR_FRAME.Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	YAW_MOTOR_FRAME.Data[4] = (uint8_t)(KP_Tmp);
	YAW_MOTOR_FRAME.Data[5] = (uint8_t)(KD_Tmp>>4);
	YAW_MOTOR_FRAME.Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	YAW_MOTOR_FRAME.Data[7] = (uint8_t)(Torque_Tmp);
  USER_CAN_TxMessage(&YAW_MOTOR_FRAME);
}
static void Damiao_Motor_Enable(){

	YAW_MOTOR_FRAME.Data[0] = 0xFF;
	YAW_MOTOR_FRAME.Data[1] = 0xFF;
	YAW_MOTOR_FRAME.Data[2] = 0xFF;
	YAW_MOTOR_FRAME.Data[3] = 0xFF;
	YAW_MOTOR_FRAME.Data[4] = 0xFF;
	YAW_MOTOR_FRAME.Data[5] = 0xFF;
	YAW_MOTOR_FRAME.Data[6] = 0xFF;
	YAW_MOTOR_FRAME.Data[7] = 0xFC;

  USER_CAN_TxMessage(&YAW_MOTOR_FRAME);
}

static void Dji_Motor_Pitch_Can_Send(int16_t current_pitch){
  PITCH_MOTOR_FRAME.Data[0] = 0;
  PITCH_MOTOR_FRAME.Data[1] = 0;
  PITCH_MOTOR_FRAME.Data[2] = 0;
  PITCH_MOTOR_FRAME.Data[3] = 0;
  PITCH_MOTOR_FRAME.Data[4] = 0;
  PITCH_MOTOR_FRAME.Data[5] = 0;
  PITCH_MOTOR_FRAME.Data[6] = current_pitch >> 8;
  PITCH_MOTOR_FRAME.Data[7] = current_pitch;
  USER_CAN_TxMessage(&PITCH_MOTOR_FRAME);
}

static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left,int16_t current_Shoot_Right,int16_t current_Shoot_Pull){
    ALLShootTxFrame.Data[0] = current_Shoot_Left>>8;
    ALLShootTxFrame.Data[1] = current_Shoot_Left;
    ALLShootTxFrame.Data[2] = current_Shoot_Right>>8;
    ALLShootTxFrame.Data[3] = current_Shoot_Right;
    ALLShootTxFrame.Data[4] = current_Shoot_Pull>>8;
    ALLShootTxFrame.Data[5] = current_Shoot_Pull;
    ALLShootTxFrame.Data[6] = 0;
    ALLShootTxFrame.Data[7] = 0;
    USER_CAN_TxMessage(&ALLShootTxFrame);
}
static void Dji_Motor_Chassis_Can_Send(int16_t current_id_1,int16_t current_id_2,int16_t current_id_3,int16_t current_id_4){
    ALLChassisTxFrame.Data[0] = current_id_1  >>8;
    ALLChassisTxFrame.Data[1] = current_id_1;
    ALLChassisTxFrame.Data[2] = current_id_2  >>8;
    ALLChassisTxFrame.Data[3] = current_id_2;
    ALLChassisTxFrame.Data[4] = current_id_3  >>8;
    ALLChassisTxFrame.Data[5] = current_id_3;
    ALLChassisTxFrame.Data[6] = current_id_4  >>8;
    ALLChassisTxFrame.Data[7] = current_id_4;

    USER_CAN_TxMessage(&ALLChassisTxFrame);
}