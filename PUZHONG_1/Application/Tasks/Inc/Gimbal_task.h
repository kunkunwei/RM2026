#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

// #include "can_task.h"
#include "old_pid.h"
// #include "chassis_behaviour.h"
#include "user_lib.h"
//
#define GIMBAL_WZ_RC_SEN 0.0f
#define GIMBAL_PITCH_RC_SEN 0.0f
//

////////////////////////////
#define SHOOT_SPD 8.0f
#define SHOOT_CHANNEL_TO_SPEED 1.0/660.0f              //660 -> 1rad
///////////////////////////////
#define PID_SHOOT_RIGHT_MOTOR_KP        900.0f
#define PID_SHOOT_RIGHT_MOTOR_KI        0.3f
#define PID_SHOOT_RIGHT_MOTOR_KD        0.0f    
#define PID_SHOOT_RIGHT_MOTOR_MAXOUT    5000.0f
#define PID_SHOOT_RIGHT_MOTOR_MAXI      100.0f

#define PID_SHOOT_LEFT_MOTOR_KP         900.0f   
#define PID_SHOOT_LEFT_MOTOR_KI         0.3f
#define PID_SHOOT_LEFT_MOTOR_KD         0.0f  
#define PID_SHOOT_LEFT_MOTOR_MAXOUT     5000.0
#define PID_SHOOT_LEFT_MOTOR_MAXI       100.0f   

#define PID_SHOOT_LOCK_KP 3500.0f
/////////////////////////////////
#define PID_SHOOT_PULL_MOTOR_KP         280.0f
#define PID_SHOOT_PULL_MOTOR_KI         1.0f
#define PID_SHOOT_PULL_MOTOR_KD         0.0f
#define PID_SHOOT_PULL_MOTOR_MAXOUT     10000.0f
#define PID_SHOOT_PULL_MOTOR_MAXI       7000.0f
////////////////////////////////////
/////////////////////////////////
#define PID_PITCH_MOTOR_VEL_KP         0.0f
#define PID_PITCH_MOTOR_VEL_KI         0.00f
#define PID_PITCH_MOTOR_VEL_KD         0.0f
#define PID_PITCH_MOTOR_VEL_MAXOUT     0.0f
#define PID_PITCH_MOTOR_VEL_MAXI       0.0f
////////////////////////////////////
/////////////////////////////////
#define PID_PITCH_MOTOR_POS_KP         16888.0f
#define PID_PITCH_MOTOR_POS_KI         10.0f
#define PID_PITCH_MOTOR_POS_KD         6.66f
#define PID_PITCH_MOTOR_POS_MAXOUT     15000.0f
#define PID_PITCH_MOTOR_POS_MAXI       2800.0f
////////////////////////////////////
/////////////////////////////////
#define PID_YAW_MOTOR_POS_KP         0.0f
#define PID_YAW_MOTOR_POS_KI         0.0f
#define PID_YAW_MOTOR_POS_KD         0.0
#define PID_YAW_MOTOR_POS_MAXOUT     0.0f
#define PID_YAW_MOTOR_POS_MAXI       0.0f
////////////////////////////////////
//
// typedef struct
// {
//     dji_motor_measure_t* pull_motor;
//     dji_motor_measure_t* shoot_motor_left;
//     dji_motor_measure_t* shoot_motor_right;
//
//     float shoot_speed;
//
//     PidTypeDef Pull_PID;
//     PidTypeDef Shoot_right_PID;
//     PidTypeDef Shoot_left_PID;
//
//     bool_t shoot_ready_flag;
// }gimbal_shoot_t;
//
// typedef struct
// {
//     dm_motor_measure_t* yaw_motor;
//     dji_motor_measure_t* pitch_motor;
//
//     float pitch_target_pos;
//
//     float yaw_target_pos;
//
//     //PidTypeDef Pitch_Vel_PID;
//     PidTypeDef Pitch_Pos_PID;
//
//     PidTypeDef Yaw_Pos_PID;
//     //PidTypeDef Yaw_Vel_PID;
//
//     first_order_filter_type_t filter_rc_pitch_vel_set;
//     first_order_filter_type_t filter_rc_yaw_vel_set;
//
//     bool_t calibrate_warning;
// }gimbal_pos_control_t;
//
// typedef struct
// {
//     gimbal_pos_control_t gimbal_pos;
//     gimbal_shoot_t gimbal_shoot;
//
//     const control_mode_t* rc_control_mode;
//     const Remote_Info_Typedef *gimbal_RC;
// }gimbal_t;
//
// extern gimbal_t gimbal;
// void Gimbal_Task(void const * argument);

#endif 