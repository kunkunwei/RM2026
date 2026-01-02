#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

// #include "can_task.h"
#include "old_pid.h"
// #include "chassis_behaviour.h"
#include "mymotor.h"
#include "stepper_can.h"
#include "user_lib.h"
//
#define GIMBAL_WZ_RC_SEN 0.0f
#define GIMBAL_PITCH_RC_SEN 0.0f
//////////////////////////////////////
/* 遥控器开关定义 */
#define RIGHT_SWITCH 0 // 右拨杆
#define LEFT_SWITCH  1 // 左拨杆
/* 遥控器通道定义 */
#define RC_RIGHT_X_CH 0 // 右摇杆X轴通道
#define RC_RIGHT_Y_CH 1 // 右摇杆Y轴通道
#define RC_LEFT_X_CH  2 // 左摇杆X轴通道
#define RC_LEFT_Y_CH  3 // 左摇杆Y轴通道
#define RC_ROLL_CH    4 // 滚转通道
///////////////////////
/* 云台遥控灵敏度设置 */
#define GIMBAL_WZ_RC_SEN    -0.000006f // YAW轴遥控灵敏度
#define GIMBAL_PITCH_RC_SEN 0.000007f  // PITCH轴遥控灵敏度
#define GIMBAL_ROLL_RC_SEN 0.000007f  // ROLL轴遥控灵敏度
////////////////////////////
/* PITCH轴角度限制 */
#define MAX_PITCH_RAD 0.27f  // 最大俯仰角度（弧度）
#define MIN_PITCH_RAD -0.52f // 最小俯仰角度（弧度）
/* YAW轴角度限制 */
#define MAX_YAW_RAD 0.28f  // 最大俯仰角度（弧度）
#define MIN_YAW_RAD -0.28f // 最小俯仰角度（弧度）
///////////////////////////////
/*PITCH轴校准的目标位置*/
#define PITCH_CALI_POS_1 0.0f // PITCH轴校准目标位置（弧度）
/*YAW轴校准的目标位置*/
#define YAW_CALI_POS_1 0.0f  // YAW轴校准目标
///////////////////////////////
/*角度转换系数*/
#define RAD_TO_DEG 57.295779f   // 弧度转角度系数
#define DEG_TO_RAD 0.017453292f // 角度转弧度系数
///////////////////////////////
/* 发射系统参数设置 */
#define SHOOT_SPD              750.0f       // 发射轮目标转速
#define SHOOT_CHANNEL_TO_SPEED 1.0 / 660.0f // 通道值转换为速度的系数（660->1rad）

/* 拨弹轮目标位置设置 */
#define SHOOT_PULL_TOWARD_POSTIVE -3.0f * PI // 拨弹轮正向转动目标位置
#define SHOOT_PULL_TOWARD_NEGTIVE 3.0f * PI  // 拨弹轮反向转动目标位置
///////////////////////////////
/* 右发射轮电机PID参数 */
#define PID_SHOOT_RIGHT_MOTOR_KP     500.0f   // 比例系数
#define PID_SHOOT_RIGHT_MOTOR_KI     5.000f   // 积分系数
#define PID_SHOOT_RIGHT_MOTOR_KD     5.0f     // 微分系数
#define PID_SHOOT_RIGHT_MOTOR_MAXOUT 12000.0f // PID最大输出
#define PID_SHOOT_RIGHT_MOTOR_MAXI   1200.0f  // PID最大积分输出

/* 左发射轮电机PID参数 */
#define PID_SHOOT_LEFT_MOTOR_KP     500.0f  // 比例系数
#define PID_SHOOT_LEFT_MOTOR_KI     5.0f    // 积分系数
#define PID_SHOOT_LEFT_MOTOR_KD     5.0f    // 微分系数
#define PID_SHOOT_LEFT_MOTOR_MAXOUT 12000.0 // PID最大输出
#define PID_SHOOT_LEFT_MOTOR_MAXI   1200.0f // PID最大积分输出

/* 发射轮锁定PID参数 */
#define PID_SHOOT_LOCK_KP 3500.0f // 锁定时的比例系数
/////////////////////////////////
/* 拨弹轮电机PID参数 */
#define PID_SHOOT_PULL_MOTOR_KP     400.0f   // 比例系数
#define PID_SHOOT_PULL_MOTOR_KI     0.0f     // 积分系数
#define PID_SHOOT_PULL_MOTOR_KD     0.0f     // 微分系数
#define PID_SHOOT_PULL_MOTOR_MAXOUT 10000.0f // PID最大输出
#define PID_SHOOT_PULL_MOTOR_MAXI   7000.0f  // PID最大积分输出
////////////////////////////////////

/* PITCH轴速度环PID参数 */
#define PITCH_SPEED_PID_KP       2.66f  // 比例系数
#define PITCH_SPEED_PID_KI       0.005f // 积分系数
#define PITCH_SPEED_PID_KD       0.0f   // 微分系数
#define PITCH_SPEED_PID_MAX_OUT  5.0f   // PID最大输出
#define PITCH_SPEED_PID_MAX_IOUT 0.5f   // PID最大积分输出

/* PITCH轴角度环PID参数（基于陀螺仪解算角度） */
#define PITCH_GYRO_ABSOLUTE_PID_KP       20.0f // 比例系数
#define PITCH_GYRO_ABSOLUTE_PID_KI       0.08f  // 积分系数
#define PITCH_GYRO_ABSOLUTE_PID_KD       0.0f  // 微分系数
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT  10.0f // PID最大输出
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f  // PID最大积分输出

/* YAW轴速度环PID参数 */
#define YAW_SPEED_PID_KP       1.2f  // 比例系数
#define YAW_SPEED_PID_KI       0.005f // 积分系数
#define YAW_SPEED_PID_KD       0.0f   // 微分系数
#define YAW_SPEED_PID_MAX_OUT  5.0f   // PID最大输出
#define YAW_SPEED_PID_MAX_IOUT 0.5f   // PID最大积分输出

/* YAW轴角度环PID参数（基于陀螺仪解算角度） */
#define YAW_GYRO_ABSOLUTE_PID_KP       10.0f // 比例系数
#define YAW_GYRO_ABSOLUTE_PID_KI       0.0f  // 积分系数
#define YAW_GYRO_ABSOLUTE_PID_KD       0.0f  // 微分系数
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT  10.0f // PID最大输出
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f  // PID最大积分输出
////////////////////////////////////
/**
 * @brief 云台工作模式枚举
 * @details 定义云台的各种工作状态和控制模式
 */
typedef enum {
    GIMBAL_MOD_NO_FORCE  = 0, // 无力模式，电机无输出，默认模式
    GIMBAL_MOD_SLOW_CALI = 1, // 慢速校准模式，用于初始化校准
    GIMBAL_MOD_NORMAL    = 2, // 正常控制模式，手动遥控
    GIMBAL_MOD_AutoAim   = 3, // 自动瞄准模式，视觉辅助


    GIMBAL_MOD_CONTROL_BY_PC = 4, // PC控制模式，上位机控制

} gimbal_mod_e;
typedef struct
{
    dji_motor_measure_t *pull_motor;        // 拨弹轮电机测量数据指针
    dji_motor_measure_t* shoot_motor_left;
    dji_motor_measure_t* shoot_motor_right;

    float shoot_left_rad;                   // 左发射轮弧度位置
    float shoot_right_rad;                  // 右发射轮弧度位置
    float shoot_pull_rad;                   // 拨弹轮弧度位置

    float shoot_speed;              // 发射轮当前转速

    TickType_t no_block_update_time; // 无堵转时间更新
    TickType_t block_time;           // 堵转时间记录

    float shoot_target_speed;       // 发射轮目标转速
    float pull_target_speed;         // 拨弹轮目标转速

    PidTypeDef Pull_PID;        // 拨弹轮PID控制器
    PidTypeDef Shoot_right_PID;
    PidTypeDef Shoot_left_PID;

    bool locker;
    bool pull_is_block;
    bool shoot_ready_flag;
    bool shoot;
}gimbal_shoot_t;

typedef struct
{
    Stepper_motor_measure_t* yaw_motor_measure;
    Stepper_motor_measure_t* pitch_motor_measure;

    float pitch_absolute_pos; // PITCH轴绝对位置
    float yaw_absolute_pos;   // YAW轴绝对位置

    float pitch_relatiive_pos; // PITCH轴相对位置
    float yaw_relattive_pos;   // YAW轴相对位置

    float pitch_target_pos;      // PITCH轴目标位置
    float last_pitch_target_pos; // PITCH轴上次目标位置
    float yaw_target_pos;        // YAW轴目标位置
    float last_yaw_target_pos;   // YAW轴上次目标位置

    float init_pitch_pos; // PITCH轴初始位置
    float init_yaw_pos;   // YAW轴初始位置

    PidTypeDef pitch_motor_gyro_pid;           // PITCH轴速度环PID控制器
    PidTypeDef pitch_motor_absolute_angle_pid; // PITCH轴角度环PID控制器

    PidTypeDef yaw_motor_gyro_pid;     // YAW轴速度环PID控制器
    PidTypeDef yaw_absolute_angle_pid; // YAW轴角度环PID控制器

    first_order_filter_type_t filter_rc_pitch_vel_set;
    first_order_filter_type_t filter_rc_yaw_vel_set;

    bool_t calibrate_warning;
    float add_yaw;   // YAW轴附加角度
    float add_pitch; // PITCH轴附加角度
    float tmp;       // 临时变量
}gimbal_pos_control_t;

typedef struct
{
    gimbal_pos_control_t gimbal_pos;
    gimbal_shoot_t gimbal_shoot;

    gimbal_mod_e last_gimbal_mod; // 上一次工作模式
    gimbal_mod_e gimbal_mod;      // 当前工作模式

    const Remote_Info_Typedef *gimbal_RC;
    const INS_Info_Typedef *ins_info;   // 惯性导航系统信息指针
}gimbal_t;

extern gimbal_t gimbal;
void Gimbal_Task(void const * argument);

#endif 