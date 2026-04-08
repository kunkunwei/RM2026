

#include "observe_task.h"
#include "Gimbal_task.h"
#include "sbus_remote.h"


// extern const fp32 *local_imu_gyro;	  // IMU陀螺仪数据指针
// extern SBUS_Remote_Info_Typedef sbus_remote_ctrl;

void ObserveTask(void const * argument)
{

	const gimbal_t *locol_gimbal=get_gimbal_point();
  TickType_t systick = 0;  // 系统时钟计数
	bool enable = false;
  while(1)
	{
  	systick = osKernelSysTick();
#ifdef USE_SBUS_PROTOCOL
  	// SBUS_Connection_Monitor(&sbus_remote_ctrl);
// #else
#endif
  	// // 检测
  	// if( gimbal.gimbal_pos.yaw_motor_measure->err != 1){
  	// 	// uart_printf(&huart1,"YAW:离线 ERR:%d\r\n",gimbal.gimbal_pos.yaw_motor_measure->err);
  	// 	Damiao_Motor_Enable(1);
  	// 	osDelay(10);
  	// }
  	if (locol_gimbal->gimbal_pos.pitch_motor_measure->tmos_tmper!=0&&enable==false)
  	{
  		Damiao_Motor_Enable();
  		enable=true;
  	}
  	if( locol_gimbal->gimbal_pos.pitch_motor_measure->err != 1){
  		// uart_printf(&huart1,"PITCH:离线 ERR:%d\r\n",gimbal.gimbal_pos.pitch_motor_measure->err);
  		Damiao_Motor_Enable();
  		// osDelay(10);
  	}
		osDelayUntil(&systick,50);  // 50ms周期
	}
}

