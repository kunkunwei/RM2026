/**
  ******************************************************************************
  * @file           : observe_task.c
  * @brief          : 观测任务实现文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 实现轮足机器人状态观测，包括速度估计、打滑检测等功能
  *                  采用卡尔曼滤波融合IMU加速度和轮速计数据
  ******************************************************************************
  */

#include "observe_task.h"

#include "Gimbal_task.h"
// #include "motor.h"
#include "sbus_remote.h"
#include "usart.h"
#include "vofa.h"


/* 卡尔曼滤波器实例 */
KalmanFilter_Info_TypeDef vaEstimateKF;	  


extern const fp32 *local_imu_gyro;	  // IMU陀螺仪数据指针
extern SBUS_Remote_Info_Typedef sbus_remote_ctrl;
/**
 * @brief  观测任务主函数
 * @param  argument FreeRTOS任务参数(未使用)
 * @retval None
 * @note   以5ms周期运行，进行速度观测和状态估计
 */
void ObserveTask(void const * argument)
{

	// xvEstimateKF_Init(&vaEstimateKF);  // 初始化卡尔曼滤波器
	const  gimbal_t* locaal_gimbal=get_gimbal_point();
  TickType_t systick = 0;  // 系统时钟计数
  while(1)
	{
  	systick = osKernelSysTick();
#ifdef USE_SBUS_PROTOCOL
  	SBUS_Connection_Monitor(&sbus_remote_ctrl);
// #else
#endif
  	// // 检测
  	if( locaal_gimbal->gimbal_pos.yaw_motor_measure->err != 1){
  		// uart_printf(&huart1,"YAW:离线 ERR:%d\r\n",gimbal.gimbal_pos.yaw_motor_measure->err);
  		Damiao_Motor_Enable(1);
  		osDelay(10);
  	}
  	if( locaal_gimbal->gimbal_pos.pitch_motor_measure->err != 1){
  		// uart_printf(&huart1,"PITCH:离线 ERR:%d\r\n",gimbal.gimbal_pos.pitch_motor_measure->err);
  		Damiao_Motor_Enable(2);
  		osDelay(10);
  	}
  	if( locaal_gimbal->gimbal_pos.roll_motor_measure->err != 1){
  		// uart_printf(&huart1,"ROLL:离线 ERR:%d\r\n",gimbal.gimbal_pos.roll_motor_measure->err);
  		Damiao_Motor_Enable(3);
  		osDelay(10);
  	}
		// osDelayUntil(&systick,5);  // 5ms周期延时
  	// osDelay(20);
  	osDelayUntil(&systick, 50); // 50ms周期控制
	}
}
