/**
	****************************(C) COPYRIGHT 2016 DJI****************************
	* @file       chassis_behaviour.c/h
	* @brief      完成底盘行为任务。
	* @note
	* @history
	*  Version    Date            Author          Modification
	*  V1.0.0     Dec-26-2018     RM              1. 完成
	*  v2.0.0     Nov-05-2023     pxx             刚起步
	*
	@verbatim
	==============================================================================

	==============================================================================
	@endverbatim
	****************************(C) COPYRIGHT 2016 DJI****************************
	*/
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "main.h"
#include "remote_control.h"
#include "old_pid.h"
// #include "CAN_Receive.h"

#include "pc_uart_ctrl.h"
#include "user_lib.h"
// #include "observe_task.h"

// 前向声明
//  struct JumpController_t;
/*底盘CAN_ID
			 前
		1     4
	左  2         1  右
		2     3
			 后
*/

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
/////////////////////////////////////////////////////////////////
// 云台YAW的遥控器通道号码
#define CHASSIS_X_CHANNEL 0
// 云台PITCH的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 1
// 底盘旋转的遥控器通道号码
#define CHASSIS_L_CHANNEL 2
// 前后的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL 3
// 功能开关通道号
#define MODE_CHANNEL 0
#define FUNCTION_CHANNEL 1
///////////////////////////////////////////////////////////////
// 遥控遥感死区限制
#define CHASSIS_RC_DEADLINE 10

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.003f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.003f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.008f
////////////////////////////////////////////////////////////////
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_SPEED_NUM 0.05f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.1666666667f



// 底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// 底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

#define yaw_dir 1.0f // Yaw轴正方向，逆时针为正
#define MOTOR_RPM_TO_ROTATE 0.10471975512f // 2PI /60

// // 底盘电机最大速度
// #define MAX_WHEEL_SPEED 1.0f
// // 底盘运动过程最大前进速度
// #define NORMAL_MAX_CHASSIS_SPEED_X 1.3f
// // 底盘设置旋转速度
// #define CHASSIS_WZ_SET_SCALE 0.0f

//////////////////////////////////////////////////////////
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define MOTOR_DISTANCE_TO_CENTER 0.2f
//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
// #define M3508_MOTOR_RPM_TO_VECTOR 0.0006237146233552417758135
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 3.6f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 4.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.9f
#define NORMAL_MAX_CHASSIS_SPEED_WZ 3.0f

//小陀螺旋转速度
#define ROTATION_SPEED_MAX 5.5f
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f //510.3 346.1

//////////////////////////////////////////////////////////

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 17000.0f
#define M3505_MOTOR_SPEED_PID_KI 12.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 8000.0f	//16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 8.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.15f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//////////////////////////////////////////////////////////

// 获取姿态角指针地址后，对应姿态角的地址偏移量 fp32类型
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 1
#define INS_GYRO_Y_ADDRESS_OFFSET 0
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 1
#define INS_ACCEL_Y_ADDRESS_OFFSET 0
#define INS_ACCEL_Z_ADDRESS_OFFSET 2
typedef enum
{
	CHASSIS_FORCE_RAW = 0,								// 底盘开环控制
	CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW = 1, // 底盘跟随云台（云台正方向为机器人正方向）
	CHASSIS_GIMBAL_FOLLOW_CHASSIS = 2,		// 云台跟随底盘（底盘正方向为机器人正方向）
	CHASSIS_VECTOR_NO_FOLLOW_YAW=3,				// 底盘向量控制但不跟随云台（遥控器前进方向为机器人正方向）
} chassis_mode_e;
typedef struct
{
	const dji_motor_measure_t *chassis_motor_measure;
	float accel;
	float speed;
	float speed_set;
	int16_t target_current;
} Chassis_Motor_t;
typedef struct
{
	PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
	PidTypeDef chassis_angle_pid;              //底盘跟随角度pid
} Chassis_Pid_t;
typedef struct
{
	float vx;                     //底盘设定速度 前进方向 前为正，单位 m/s
	float vy;                     //底盘设定速度 左右方向 左为正，单位 m/s
	float wz;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
    float chassis_yaw_set;		      // 设置底盘yaw转向期望绝对角度

	first_order_filter_type_t chassis_cmd_slow_set_vx;
	first_order_filter_type_t chassis_cmd_slow_set_vy;
	first_order_filter_type_t chassis_cmd_slow_set_wz;

	ramp_function_source_t rotation_ramp_wz;   //小陀螺斜坡函数缓启动 停止
	bool rotation_diraction;                 //小陀螺旋转的方向


} Chassis_set_t;
typedef struct
{
	float vx;                     //底盘速度 前进方向 前为正，单位 m/s
	float vy;                     //底盘速度 左右方向 左为正，单位 m/s
	float wz;                     //底盘旋转角速度，逆时针为正 单位 rad/s
	float chassis_yaw_absolute;		      // 底盘yaw绝对角度
	float chassis_yaw_relative;		      // 底盘yaw相对云台角度
	float gimbal_yaw_relative;		      // 云台yaw相对底盘角度

} Chassis_ref_t;
typedef struct
{
	chassis_mode_e chassis_mode;			// 底盘控制状态机
	chassis_mode_e last_chassis_mode;		// 底盘上次控制状态机
	int8_t last_fanction_channel;                //上一次遥控器功能开关所在的位置
	int8_t last_normol_channel;                //上一次遥控器开关所在的位置
} Chassis_mode_t;

typedef struct
{
	const PC_Ctrl_Info_t *VT03_ctrl;					// 图传遥控器指针
	const Remote_Info_Typedef *chassis_RC;				//	DT7遥控器指针
	const float *chassis_INS_angle;								// 获取陀螺仪解算出的欧拉角指针
	const float *chassis_imu_gyro;								// 获取角加速度指针
	const float *chassis_imu_accel;								// 获取加速度指针

	Chassis_mode_t mode;						// 底盘控制状态机相关变量
	bool VT03_fn1_last;                   // 上一次图传链路fn_1按键状态
	bool VT03_fn2_last;                   // 上一次图传链路fn_2按键状态
	bool VT03_pause_last;                 // 上一次图传链路pause按键状态
	bool auto_aim_mode;                   // 自动瞄准模式
	bool spinning_mode;					  // 小陀螺模式
	bool last_spinning_mode;					  // 上次小陀螺模式

	Chassis_Motor_t chassis_motor[4];          //底盘电机数据
	dm_motor_measure_t *yaw_motor_measure;    // YAW轴电机测量数据指针
	float gimbal_yaw_rad;                      // 云台yaw绝对角度，底盘跟随参考

	ramp_function_source_t rotation_ramp_wz;   //小陀螺斜坡函数缓启动 停止

	Chassis_Pid_t chassis_pid;                //底盘PID数据
	Chassis_set_t state_set;                //底盘状态设定值
	Chassis_ref_t state_ref;                //底盘状态反馈值

} chassis_move_t;

//
// 获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void);
const Chassis_ref_t *get_chassis_ref_point(void);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);

#endif
