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

#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "main.h"

// #include "chassis_task.h"

//////摇杆区域 S//////
#define RC_RIGHT_X_CH 0
#define RC_RIGHT_Y_CH 1
#define RC_LEFT_X_CH 2
#define RC_LEFT_Y_CH 3
#define RC_ROLL_CH 4

#define CHASSIS_X_CHANNEL   RC_LEFT_Y_CH
#define CHASSIS_Y_CHANNEL   RC_LEFT_X_CH
#define CHASSIS_WZ_CHANNEL  RC_RIGHT_X_CH

#define GIMBAL_PITCH_CHANNEL RC_RIGHT_Y_CH
#define GIMBAL_YAW_CHANNEL RC_RIGHT_X_CH
#define GIMBAL_SHOOT_CHANNEL RC_ROLL_CH
//摇杆END////////////////////

//拨杆 S//////////////
#define MODE_CHANNEL 0
#define FUNCTIONAL_CHANNEL 1
//拨杆END///////////////////

#define TMP_DEFINE 
//
///////////////MODE TYPEDEF START//////////////////
typedef enum
{
	CHASSIS_FORCE_RAW,				        // 底盘开环控制
	CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW, // 底盘跟随云台,
	CHASSIS_VECTOR_NO_FOLLOW_YAW,	    // 底盘不跟随云台,云台固定角度
} chassis_mode_e;

typedef enum
{
  GIMBAL_DISABLE,
	GIMBAL_CALIBRATE,
  
  GIMBAL_FOLLOW_CHASSIS,
  GIMBAL_FREE_CONTROL,

} gimbal_mode_e;

typedef struct
{
  //mode s
  chassis_mode_e chassis_mode;
  chassis_mode_e last_chassis_mode;

  gimbal_mode_e gimbal_mode;
  gimbal_mode_e last_gimbal_mode;
  
  int8_t last_rc_func_state;
  //mode end

  //flag s
  bool_t self_turn_flag;

  int16_t last_shoot_channel;
  int16_t shoot_channel;
  bool_t shoot_flag;//摩擦轮开转
  bool_t fire_flag;//kaihuo
  
  bool_t auto_shoot_flag;
  //flag e
} control_mode_t;
///////////////MODE TYPEDEF END//////////////////

extern control_mode_t control_mode;
const control_mode_t* get_rc_control_mode();

void chassis_behaviour_mode_set(Remote_Info_Typedef const* remote_ctrl);
void gimbal_behaviour_mode_set( Remote_Info_Typedef const* remote_ctrl);

#endif
