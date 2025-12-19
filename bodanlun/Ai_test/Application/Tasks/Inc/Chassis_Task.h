// /**
//   ****************************(C) COPYRIGHT 2016 DJI****************************
//   * @file       chassis_behaviour.c/h
//   * @brief      完成底盘行为任务。
//   * @note
//   * @history
//   *  Version    Date            Author          Modification
//   *  V1.0.0     Dec-26-2018     RM              1. 完成
//   *  v2.0.0     Nov-05-2023     pxx             刚起步
//   *
//   @verbatim
//   ==============================================================================

//   ==============================================================================
//   @endverbatim
//   ****************************(C) COPYRIGHT 2016 DJI****************************
//   */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "chassis_behaviour.h"
#include "main.h"
#include "remote_control.h"
#include "old_pid.h"
#include "user_lib.h"
//#include "observe_task.h"
// #include "pid.h"

// /*底盘CAN_ID
//2         1
//3         4

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.005f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_WZ_RC_SEN 0.004f
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

#define CHASSIS_ROS_TO_WZ 0.0021f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f
//底盘电机最大速度
#define MAX_WHEEL_SPEED 3.0f

// #define CHASSIS_ACCEL_X_NUM 0.1666666667f
// #define CHASSIS_SPEED_NUM 0.05f
// // #define CHASSIS_ACCEL_Y_NUM 0.3333333333f

// 遥控遥感死区限制
#define CHASSIS_RC_DEADLINE 10

// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// 底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f

// 底盘电机最大速度
// #define MAX_WHEEL_SPEED 1.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Z 4.0f

#define CHASSIS_MOTOR_W_TO_VECTOR_SEN WHEEL_R
#define WHEELR 0.075f  //7.5cm

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//小陀螺旋转速度
#define ROTATION_SPEED_MAX 3.0f
#define ROTATION_SPEED_ADD_VALUE ROTATION_SPEED_MAX/3

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f
//单独的角速度环
#define ROS_WZ_PID_KP 2.0f
#define ROS_WZ_PID_KI 0.004f
#define ROS_WZ_PID_KD 0.1f
#define ROS_WZ_PID_MAX_OUT 10.0f
#define ROS_WZ_PID_MAX_IOUT 0.5f
typedef struct 
{
	float vx;
	float vy;
	float wz;
}chassis_vel_t;

typedef struct
{
	const Remote_Info_Typedef *chassis_RC;	  // 底盘使用的遥控器指针
	const control_mode_t* rc_control_mode;

	//
	dji_motor_measure_t* chassis_motor[4];
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	chassis_vel_t vel_ref;
	chassis_vel_t vel_set;
	//

	float chassis_yaw;
	PidTypeDef chassis_ros_wz_pid;  //底盘ROS_角速度_PID

	first_order_filter_type_t chassis_cmd_slow_set_vx;
  	first_order_filter_type_t chassis_cmd_slow_set_vy;
  	first_order_filter_type_t chassis_cmd_slow_set_wz;
	first_order_filter_type_t state_xdot_filter;
	ramp_function_source_t rotation_ramp_wz;   //小陀螺斜坡函数缓启动 停止
 	bool_t rotation_diraction;                 //小陀螺旋转的方向

	float vx_from_ros;
	float vy_from_ros;
	float wz_from_ros;
	float kilometer;

  	fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  	fp32 vx_min_speed;  //前进方向最小速度 单位m/s
  	fp32 vy_max_speed;  //左右方向最大速度 单位m/s
  	fp32 vy_min_speed;  //左右方向最小速度 单位m/s
	
	fp32 tmp;
} chassis_move_t;

extern void Chassis_Task(void const *pvParameters);

extern chassis_move_t chassis_move;
const chassis_move_t* get_chassis_point();
#endif
