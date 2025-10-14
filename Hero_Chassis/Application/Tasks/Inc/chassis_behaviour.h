/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  v2.0.0     Nov-05-2023     pxx             刚起步
  *  v3.0.0     2025-01-18      AI              修复循环包含问题
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
#include "remote_control.h"

// 前向声明，避免循环包含
// typedef struct chassis_move_t chassis_move_t;
// typedef struct Remote_Info_Typedef Remote_Info_Typedef;

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
  CHASSIS_ZERO_FORCE,                  //底盘无力
  CHASSIS_NO_MOVE,                     //底盘保持不动
  CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,  //正常步兵底盘跟随云台
  CHASSIS_NO_FOLLOW_YAW,               //底盘不跟随YAW角度，角度是开环的，但前后左右是有速度环
  CHASSIS_OPEN,                        //遥控器的值乘以比例直接发送到can总线上

  CHASSIS_ROTATION,                    //底盘小陀螺
  CHASSIS_ROTATION_EXIT                //小陀螺退出状态机
} chassis_behaviour_e;
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, //云台无力
    GIMBAL_INIT,           //云台初始化
    GIMBAL_CALI,           //云台校准
    GIMBAL_ABSOLUTE_ANGLE, //云台陀螺仪绝对角度控制
    GIMBAL_RELATIVE_ANGLE, //云台电机编码值相对角度控制
    GIMBAL_MOTIONLESS,     //云台在遥控器无输入一段时间后保持不动，避免陀螺仪漂移

    GIMBAL_AUTO_SHOOT,     //云台射击自动瞄准
  } gimbal_behaviour_e;

typedef struct
{
  /*行为模式 start*/
  chassis_behaviour_e chassis_behaviour_mode;
  chassis_behaviour_e last_chassis_behaviour_mode;

  gimbal_behaviour_e gimbal_behaviour_mode;
  gimbal_behaviour_e last_gimbal_behaviour_mode;
  /*行为模式 end */

  /*rc 遥控器通道值 save */
  uint8_t last_rc_func_state;
  int16_t last_shoot_channel;
  int16_t shoot_channel;

  /*标志位 start */
  bool_t self_turn_flag;

  bool_t rotation_state; //小陀螺状态
  bool_t shoot_friction_flag;//摩擦轮开转
  bool_t shoot_fire_flag;//开火
  bool_t shoot_fire_continuous_flag;//连续开火
  bool_t auto_shoot_flag;//自瞄状态

  /*标志位 end */
} control_mode_t;

///////////////MODE TYPEDEF END//////////////////
///
#define CHASSIS_OPEN_RC_SCALE 10 //在chassis_open 模型下，遥控器乘以该比例发送到can上
extern control_mode_t control_mode;
const control_mode_t* get_rc_control_mode();

// void chassis_behaviour_mode_set(Remote_Info_Typedef const* remote_ctrl);
// void gimbal_behaviour_mode_set( Remote_Info_Typedef const* remote_ctrl);

// extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

//小陀螺控制云台模式为absolute
extern bool_t rotation_cmd_gimbal_absolute(void);
#endif
