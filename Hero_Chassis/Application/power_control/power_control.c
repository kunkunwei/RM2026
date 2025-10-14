#include "power_control.h"
// #include "referee.h"
#include "arm_math.h"
// #include "detect_task.h"
// #include "remote_control.h"
#include "old_pid.h"
// #include "voltage_task.h"

cap_measure_t cap_measure; // capacitor data structure
//extern chassis_move_t chassis_move;
// extern RC_ctrl_t rc_ctrl;
uint8_t cap_state = 0;
void chassis_power_control(chassis_move_t *chassis_power_control)
{

	uint16_t max_power_limit = 40;
	fp32 chassis_max_power = 0;
	float input_power = 0;		 // input power from battery (referee system)
	float initial_give_power[4]; // initial power from PID calculation
	float initial_total_power = 0;
	fp32 scaled_give_power[4];

	fp32 chassis_power = 0.0f;//当前功率
	fp32 chassis_power_buffer = 0.0f;//buffer

	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
	fp32 a = 1.23e-07;						 // k1
	fp32 k2 = 1.453e-07;					 // k2
	fp32 constant = 4.081f;

	//get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);//获取当前功率和当前的buffer
	chassis_power = cap_measure.current_power;
	chassis_power_buffer = 30;
	//记得加上初始化
	//old_PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
	//

	//get_chassis_max_power(&max_power_limit);//底盘功率限制
	max_power_limit = 40;
	
	//input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power
	input_power = max_power_limit ;

	//发送超电从电源管理拿到的功率？
	//CAN_CMD_CAP(input_power); // set the input power of capacitor controller
	
	//CAP STATE是？加速？
	// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
	// {
	// 	cap_state = 0;
	// }
	// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
	// {
	// 	cap_state = 1;
	// }

	if (cap_measure.cap_percent > 15)
	{
		if (cap_state == 0)
		{
			chassis_max_power = input_power + 5; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
		}
		else
		{
			chassis_max_power = input_power + 200;
		}
	}
	else
	{
		chassis_max_power = input_power;
	}


	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->chassis_motor[i]->rpm+
								k2 * chassis_power_control->chassis_motor[i]->rpm * chassis_power_control->chassis_motor[i]->rpm +
								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + 
								constant;

		if (initial_give_power < 0) // negative power not included (transitory)
			continue;
		initial_total_power += initial_give_power[i];
	}

	//抄功率了，需要缩放，最后输出是I_cmd
	//文件最后输出的就是I_cmd
	if (initial_total_power > chassis_max_power) // determine if larger than max power
	{
		fp32 power_scale = chassis_max_power / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
			if (scaled_give_power[i] < 0)
			{
				continue;
			}

			fp32 b = toque_coefficient * chassis_power_control->chassis_motor[i]->rpm;
			fp32 c = k2 * chassis_power_control->chassis_motor[i]->rpm * chassis_power_control->chassis_motor[i]->rpm - scaled_give_power[i] + constant;

			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp > 16000)//限幅
				{
					chassis_power_control->motor_speed_pid[i].out = 16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
			else
			{
				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
				if (temp < -16000)
				{
					chassis_power_control->motor_speed_pid[i].out = -16000;
				}
				else
					chassis_power_control->motor_speed_pid[i].out = temp;
			}
		}
	}
}
