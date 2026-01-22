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


#include "slip_detector.h"
#include "user_lib.h"
// #include "observe_task.h"

//前向声明
// struct JumpController_t;
/*底盘CAN_ID
		   前	
		1     4
  左  2         1  右
		2     3
		   后
*/

// 任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
#ifdef CAN1_Chassis_RC_Mod
// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 0
// 旋转的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 1
// 腿长的遥控器通道号码
#define CHASSIS_L_CHANNEL 2
// ROLL的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL 3

// 选择普通底盘状态 开关通道号
#define LEFT_1_SWITCH 0
#define LEFT_2_SWITCH 1
#define RIGHT_2_SWITCH 2
#define RIGHT_1_SWITCH 3

#else
// 前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
// 旋转的遥控器通道号码
#define CHASSIS_WZ_CHANNEL 2
// 腿长的遥控器通道号码
#define CHASSIS_L_CHANNEL 1
// ROLL的遥控器通道号码
#define CHASSIS_ROLL_CHANNEL 0

// 选择普通底盘状态 开关通道号
#define MODE_CHANNEL 0
#define FUNTION_CHANNEL 1
#endif

// 遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0033
// 遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_WZ_RC_SEN 0.00001f
#define CHASSIS_ROS_TO_WZ 0.00003f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_SPEED_NUM 0.05f
// #define CHASSIS_ACCEL_Y_NUM 0.3333333333f

// 遥控遥感死区限制
#define CHASSIS_RC_DEADLINE 10
//轮子到轮子的距离是54cm
#define MOTOR_DISTANCE_TO_CENTER 0.27f

// 底盘任务控制间隔 2ms
// #define CHASSIS_CONTROL_TIME_MS 5
#define CHASSIS_CONTROL_TIME_MS 2
// 底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
// 底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
// 底盘8009最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 10000.0f
// 8009电机的转矩常数
#define DM8009_TOR_CONSTANT 1.0f

// 8009 力矩转换为can_data的比例
//IMAX 41.044777
#define DM8009_TOR_TO_CAN_DATA 243.445f 
//9025 力矩转换为can_data的比例
//IMAX 32A
#define LK9025_TOR_TO_CAN_DATA 195.3125f

#define WHEEL_R 0.08f		//新胶轮
// #define WHEEL_R 0.0925f

#define MOTOR_RPM_TO_ROTATE 0.10471975512f // 2PI /60

// 底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
// 底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1.0f
// 底盘设置旋转速度
#define CHASSIS_WZ_SET_SCALE 0.0f

#define CHASSIS_MOVE_AHEAD 0
#define CHASSIS_MOVE_BACK 1
// 腿部初始长度
#define LEG_LENGTH_INIT 0.166f
// 腿部离地时便于缓冲需要的长度
#define LEG_LENGTH_BUFFER 0.20f
//跳跃时在空中收腿
#define LEG_LENGTH_JUMPPING 0.15f

#define LEG_LENGTH_MAX 0.28f
#define LEG_LENGTH_MIN 0.11f

//
// #define STOP_X_OFFSET -0.01f
// #define STOP_X_OFFSET -0.282f
#define STOP_X_OFFSET 0.1420f
// #define STOP_X_OFFSET 0.1410f
// 腿部长度控制PID
// #define LEG_LENGTH_PID_KP 250.0f
#define LEG_LENGTH_PID_KP 650.0f
// #define LEG_LENGTH_PID_KI 3.0f
#define LEG_LENGTH_PID_KI 2.0f
// #define LEG_LENGTH_PID_KD 24.0f
#define LEG_LENGTH_PID_KD 15.0f
#define LEG_LENGTH_PID_MAX_OUT 150.0f
#define LEG_LENGTH_PID_MAX_IOUT 30.0f

// 腿部误差控制PID  位置 (双腿协调)
#define ANGLE_ERR_PID_KP 400.0f
#define ANGLE_ERR_PID_KI 1.9f
#define ANGLE_ERR_PID_KD 30.0f
#define ANGLE_ERR_PID_MAX_OUT 75.0f
#define ANGLE_ERR_PID_MAX_IOUT 3.0f
// 腿部角度控制PID  速度 (双腿协调)
#define ANGLE_DOT_PID_KP 0.065f
#define ANGLE_DOT_PID_KI 0.0f
#define ANGLE_DOT_PID_KD 0.03f
#define ANGLE_DOT_PID_MAX_OUT 5.0f
#define ANGLE_DOT_PID_MAX_IOUT 0.0f

//roll控制pid (ROLL补偿腿的长度)
#define ROLL_CTRL_L_PID_KP 0.388f
#define ROLL_CTRL_L_PID_KI 0.00006f
#define ROLL_CTRL_L_PID_KD 0.233f
#define ROLL_CTRL_L_PID_MAX_OUT 10.0f
#define ROLL_CTRL_L_PID_MAX_IOUT 1.0f
//roll控制pid (ROLL补偿腿的力度)
#define ROLL_CTRL_F_PID_KP 0.0f
#define ROLL_CTRL_F_PID_KI 0.00f
#define ROLL_CTRL_F_PID_KD 0.000f
#define ROLL_CTRL_F_PID_MAX_OUT 20.0f
#define ROLL_CTRL_F_PID_MAX_IOUT 2.0f
// 底盘旋转跟随PID 位置环 转向
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 5.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.000f 
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 1.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 12.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 1.2f

//yaw 速度环 ，转向
#define YAW_SPEED_PID_KP 1.0f
#define YAW_SPEED_PID_KI 0.00f
#define YAW_SPEED_PID_KD 0.188f
#define YAW_SPEED_PID_MAX_OUT 10.0f
#define YAW_SPEED_PID_MAX_IOUT 0.5f

//单独的角速度环
#define ROS_WZ_PID_KP 2.0f
#define ROS_WZ_PID_KI 0.004f
#define ROS_WZ_PID_KD 0.1f
#define ROS_WZ_PID_MAX_OUT 10.0f
#define ROS_WZ_PID_MAX_IOUT 0.5f


#define RIGHT_FRONT 1
#define RIGHT_BACK 2
#define LEFT_FRONT 3
#define LEFT_BACK 4

#define MIT_KD_NORMAL 0.2f //正常
#define MIT_KD_OUTGROUND 0.4f //离地
#define MIT_KD_CXK 0.8f    //ROLL控制
#define MIT_KD_BRAKE 0.6f	//刹车

//获取姿态角指针地址后，对应姿态角的地址偏移量 fp32类型
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 1
#define INS_GYRO_Y_ADDRESS_OFFSET 0
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 1
#define INS_ACCEL_Y_ADDRESS_OFFSET 0
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

// ============ 跳跃参数（建议放在头文件） ============
#define JUMP_COOLDOWN_MS     2000.0f   // 跳跃冷却时间（2秒)

#define JUMP_LANDING_DAMPING 1.2f        // 落地阻尼系数
#define JUMP_LANDING_STIFFNESS 1078.0f    // 落地刚度系数:k = (总重量*9.8)/伸缩量
#define JUMP_PREPARE_PITCH_COMP 0.1f     // 准备阶段pitch补偿系数
//空中收腿PID
#define JUMP_LEFT_LEG_KP 700.0f
#define JUMP_LEFT_LEG_KI 0.0f
#define JUMP_LEFT_LEG_KD 10.0f
#define JUMP_LEFT_LEG_MAX_IOUT 20.0f
#define JUMP_LEFT_LEG_MAX_OUT 120.0f

#define JUMP_RIGHT_LEG_KP 710.0f
#define JUMP_RIGHT_LEG_KI 0.0f
#define JUMP_RIGHT_LEG_KD 10.0f
#define JUMP_RIGHT_LEG_MAX_IOUT 20.0f
#define JUMP_RIGHT_LEG_MAX_OUT 120.0f

//空中姿态控制PID 补偿力
#define JUMP_ROLL_KP 100.00f	//57.295780
#define JUMP_ROLL_KI 0.0f
#define JUMP_ROLL_KD 0.5f
#define JUMP_ROLL_MAX_OUT 65.0f
#define JUMP_ROLL_MAX_IOUT 1.0f
///

typedef enum
{
	CHASSIS_FORCE_RAW=0,				  // 底盘开环控制
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW=1, // 底盘跟随云台
	CHASSIS_VECTOR_NO_FOLLOW_YAW=2,	  // 底盘不跟随云台
} chassis_mode_e;

typedef struct
{
	const lk9025_motor_measure_t *wheel_motor_measure;
	float accel;
	float speed;
	float speed_set;
	int16_t give_current;
} Wheel_Motor_t; // 驱动轮电机结构体

typedef struct
{
	const dm8009_motor_measure_t *joint_motor_measure;
	int16_t offset_ecd;	    // 关节电机中值
	float angle;				// 关节角度
	float angle_dot;			// 关节转动速度
	float current_set;		// 关节电机力矩，由PID计算给出
	int16_t give_current;	// 实际由CAN通信给电调发送的电流值
	float tor_set;
} Joint_Motor_t; // 关节电机结构体

typedef struct
{
	Wheel_Motor_t wheel_motor;
	Joint_Motor_t front_joint;
	Joint_Motor_t back_joint;

	float leg_length;    	   // 摆杆长度
	float leg_length_set;
	float leg_angle;    		   // 摆杆与竖直方向的夹角
	float angle_dot;   		   // 腿部摆杆的旋转速度
	float length_dot;   		   // 腿部摆杆的长度变化速度

	float virtual_pole_force;   // 腿部五连杆机构的推力
	float virtual_pole_torque;  // 沿中心轴的力矩

	float front_joint_tor;     // 前关节电机VMC期望扭矩
	float back_joint_tor;      // 后关节电机VMC期望扭矩

	bool_t touching_ground;
	float mit_kd;
} Leg_Control_t;

typedef struct 
{
	float theta;			// 摆杆与竖直方向的夹角
	float theta_dot;		
	float theta_ddot;
	float x;				// 驱动轮位移
	float x_dot;
	float phi;			// 机体与水平方向夹角
	float phi_dot;
} Robot_Statement_t;

typedef struct chassis_task
{
	bool jump_flag;
	bool last_jump_flag;
	uint8_t jump_stage;
	float jump_comtorque[4];

	//
	float F0;
	float Tp;
	float L0;
	float Phi0;
	float Fee[2];

	//
	float takeoff_velocity_x;

	float last_jump_finish_time;
	float current_time;
	float takeoff_start_time;
	float takeoff_time;
	float landing_time;

	float landing_velocity_x;

	float landing_leg_length;
	float takeoff_leg_length ;
	float jump_height[2];

}Jump_State_t;


typedef struct
{
	const Remote_Info_Typedef *chassis_RC;	  // 底盘使用的遥控器指针
	const Chassis_RC_Info_t *chassis_can_rc_info; // 底盘使用的CAN1遥控器指针
	const float *chassis_INS_angle;	  // 获取陀螺仪解算出的欧拉角指针
	const float *chassis_imu_gyro;	  // 获取角加速度指针
	const float *chassis_imu_accel;	  // 获取加速度指针
	const SlipDetector_t * slip_detector; // 获取打滑检测器指针

	// dji_motor_measure_t *yaw_motor_measure;    // YAW轴电机测量数据指针

	chassis_mode_e chassis_mode;	  // 底盘控制状态机
	chassis_mode_e last_chassis_mode; // 底盘上次控制状态机
#ifdef CAN1_Chassis_RC_Mod
	int8_t last_chassis_funtion_1_mode;//底盘上次功能1状态机
	int8_t last_chassis_funtion_2_mode;//底盘上次功能2状态机
#else
	uint8_t last_chassis_funtion_mode;//底盘上次功能状态机
#endif
	Leg_Control_t left_leg;			  //左腿控制结构体
	Leg_Control_t right_leg;		  //右腿控制结构体


	PidTypeDef left_leg_length_pid;   //腿长控制器
	PidTypeDef right_leg_length_pid;  //腿长控制器

	PidTypeDef angle_err_pid;		  //双腿角度误差控制器
	PidTypeDef angle_dot_pid;

	PidTypeDef roll_ctrl_l_pid;		  //横滚角误差控制器
	PidTypeDef roll_ctrl_f_pid;

	PidTypeDef chassis_angle_pid;	  //底盘跟随角度pid
	PidTypeDef chassis_yaw_gyro_pid;  //底盘角速度PID

	PidTypeDef chassis_ros_wz_pid;  //底盘ROS_角速度_PID

	Robot_Statement_t state_ref;	// 机器人状态量
	Robot_Statement_t state_set;	// 机器人预期的状态
	first_order_filter_type_t chassis_cmd_slow_set_vx; // vx一阶低通滤波
	first_order_filter_type_t state_xdot_filter;
	first_order_filter_type_t chassis_cmd_brake_vx;

	bool_t touchingGroung;          // 机器人是否离地
	float err_tor;					// 误差力矩补偿
	float wheel_tor;					// 轮毂转矩
	float leg_tor;					// 髋关节转矩
	float left_compliment_tor;		// 左腿力矩补偿
	float right_compliment_tor;		// 右腿力矩补偿
	float left_support_force;        // 左腿的支持力
	float right_support_force;        // 右腿的支持力
	float ground_force;      		// 地面的支持力
	float l_force;					// 左腿腿部推力
	float r_force;					// 右腿腿部推力
	float tor_vector[4];			// 四个电机的力矩分量

	float leg_angle;					 // 腿部角度，平均值
	float leg_angle_dot;
	float leg_length;				 // 腿长，平均值
	float leg_length_set;
	float leg_length_dot;
	float leg_length_max;             // 腿部活动范围限制
	float leg_length_min;             // 间接限制了关节电机的活动范围，关节电机还要有机械限位
	float wz;						 // 底盘旋转角速度，逆时针为正 单位 rad/s
	float wz_set;				     // 底盘旋转角速度，逆时针为正 单位 rad/s
	float chassis_yaw_set;            // 设置底盘陀螺仪yaw期望角度
	float chassis_roll_set;

	float vx_max_speed;	// 前进方向最大速度 单位m/s
	float vx_min_speed;	// 前进方向最小速度 单位m/s
	float chassis_yaw;	// 陀螺仪和云台电机叠加的yaw角度
	float chassis_pitch; // 陀螺仪和云台电机叠加的pitch角度
	float chassis_roll;	// 陀螺仪和云台电机叠加的roll角度

	Jump_State_t jump_state;//跳跃的标志位
	float leg_length_in_sky;

	float left_leg_real_support;
	float right_leg_real_support;
	bool_t is_conversely;//是否倒地
	uint32_t last_out_ground_tick;
	uint32_t now_tick;

	bool_t position_control_intervention;//位置控制介入
	float mit_normal_kd;//正常行驶状态的kd设置	
	uint8_t chassis_move_toward;//刹车姿态优化逻辑
	uint8_t chassis_move_last_toward;//刹车姿态优化逻辑
	bool_t change_toward_flag;//刹车姿态优化逻辑
	
	float kilometer;
	float vx_from_ros;
	float wz_from_ros;
	bool spining_state;//旋转状态
	bool spining_flag;//旋转控制标志位
	bool last_spining_flag;
	float tmp;
} chassis_move_t;

//
//获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void);

// extern void chassis_task(void *pvParameters);
extern void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern void chassis_rc_to_control_euler(float *l_set, float *roll_set, chassis_move_t *chassis_move_rc_to_vector);

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
// // 底盘初始化，主要是pid初始化
// extern void chassis_init(chassis_move_t *chassis_move_init);
// // 底盘状态机选择，通过遥控器的开关
// extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
// // 底盘数据更新
// extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
// // 底盘状态改变后处理控制量的改变static
// void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
// // 底盘设置根据遥控器控制量
// extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
// // 底盘PID计算以及运动分解
// extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

// // 获取底盘结构体指针
// const chassis_move_t *get_chassis_control_point(void);

// // extern bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);
#endif
