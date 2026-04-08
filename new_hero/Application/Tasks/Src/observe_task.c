

#include "observe_task.h"
#include "Gimbal_task.h"
#include "sbus_remote.h"


// extern const fp32 *local_imu_gyro;	  // IMU陀螺仪数据指针
// extern SBUS_Remote_Info_Typedef sbus_remote_ctrl;

void ObserveTask(void const * argument)
{
	/* 等待云台初始化完成后再注册回调，避免访问空指针 */
	while (!is_gimbal_init_done()) osDelay(1);

	const gimbal_t *locol_gimbal=get_gimbal_point();
  TickType_t systick = 0;  // 系统时钟计数
  while(1)
	{
  	systick = osKernelSysTick();

  	// // 检测
  	if( locol_gimbal->gimbal_pos.yaw_motor_measure->err != 1){
  		// uart_printf(&huart1,"YAW:离线 ERR:%d\r\n",gimbal.gimbal_pos.yaw_motor_measure->err);
  		Damiao_Motor_Enable(CAN1_YAW_MOTOR_ID);
  	}
  	// if (locol_gimbal->gimbal_pos.pitch_motor_measure->!=0&&enable==false)
  	// {
  	// 	Damiao_Motor_Enable(2);
  	// 	enable=true;
  	// }
  	if( locol_gimbal->gimbal_pos.pitch_motor_measure->err != 1){
  		// uart_printf(&huart1,"PITCH:离线 ERR:%d\r\n",gimbal.gimbal_pos.pitch_motor_measure->err);
  		Damiao_Motor_Enable(CAN2_PITCH_MOTOR_ID);

  	}
		osDelayUntil(&systick,100);  // 100ms周期
	}
}

