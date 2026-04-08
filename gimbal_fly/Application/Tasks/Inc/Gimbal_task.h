/**
 * @file Gimbal_task.h
 * @brief 云台控制任务头文件
 * @details 包含云台PID控制、发射系统控制、模式切换等功能的定义
 * @author RM Team
 * @date 2025
 */

#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

#include "can_task.h"
#include "old_pid.h"
// #include "chassis_behaviour.h"

#include "user_lib.h"
#include "usb.h"

/////////////////////
#define USE_RC_CONTROL      // 使用遥控器控制模式
// #define USE_USART_CONTROL      // 使用串口控制模式
#define REDDUCE_K         2.0f // YAW减速系数
//////////////////////
//使用SBUS协议的时候，需要更改STM32CUBEMX里面的USART3配置，100K，8E2
//使用DBUS协议的时候，需要更改STM32CUBEMX里面的USART3配置，100K，8N1
// #define USE_SBUS_PROTOCOL  //解开这个注释来使用SBUS协议，否则使用DBUS协议
/////////////////////////

#ifdef USE_SBUS_PROTOCOL
/* 遥控器开关定义 */
#define LEFT_1_SWITCH  0 // 左1拨杆
#define LEFT_2_SWITCH 1 // 左2拨杆
#define RIGHT_1_SWITCH 3 // 右1拨杆
#define RIGHT_2_SWITCH 2 // 右2拨杆
/* 遥控器通道定义 */
#define RC_RIGHT_X_CH 0 // 右摇杆X轴通道
#define RC_RIGHT_Y_CH 1 // 右摇杆Y轴通道
#define RC_LEFT_X_CH  3 // 左摇杆X轴通道
#define RC_LEFT_Y_CH  2 // 左摇杆Y轴通道
#define RC_ROLL_CH    4 // 滚转通道
/*滚转通道数值判断界限*/
#define CH4_HIGH_VALUE 500
#define CH4_LOW_VALUE 400
///////////////////////
/* 云台遥控灵敏度设置 */
#define GIMBAL_WZ_RC_SEN    -0.000006f // YAW轴遥控灵敏度
#define GIMBAL_PITCH_RC_SEN 0.000003f  // PITCH轴遥控灵敏度
#define GIMBAL_ROLL_RC_SEN -0.000003f  // ROLL轴遥控灵敏度
#else
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
#endif



/* PITCH轴角度限制 */
#define MAX_PITCH_RAD 0.27f  // 最大俯仰角度（弧度）
#define MIN_PITCH_RAD -0.52f // 最小俯仰角度（弧度）
/* ROLL轴角度限制 */
#define MAX_ROLL_RAD 0.28f  // 最大俯仰角度（弧度）
#define MIN_ROLL_RAD -0.28f // 最小俯仰角度（弧度）
///////////////////////////////
/* 发射系统参数设置 */
#define SHOOT_SPD              750.0f       // 发射轮目标转速
#define SHOOT_CHANNEL_TO_SPEED 1.0 / 660.0f // 通道值转换为速度的系数（660->1rad）

/* 拨弹轮参数：角度用于判定一发，速度用于控制弹频 */
#define SHOOT_SINGLE_BULLET_RAD -3.0f * PI  // 单发拨弹轮累计转角阈值
#define SHOOT_PULL_FORWARD_SPEED -12.0f      // 拨弹轮正向目标角速度(rad/s)，调大可提升弹频
#define SHOOT_PULL_REVERSE_SPEED 12.0f     // 堵转解卡反向角速度(rad/s)
#define SHOOT_SINGLE_TIMEOUT_MS 900u        // 单发超时兜底，超时后强制认为一发完成
/* 拨弹轮堵转检测参数（计数基于 2ms 控制周期）*/
#define BLOCK_TRIGGER_SPEED 0.5f // rad/s，拨弹轮速度低于此值视为可能堵转
#define BLOCK_TIME_COUNT 350u    // 350×2ms = 700ms 持续慢速后判定卡弹
#define REVERSE_TIME_COUNT 300u  // 300×2ms = 600ms 反转持续时间后自动复位
/* 连发长按阈值（基于 osKernelSysTick 计数，默认 1tick=1ms）*/
#define SHOOT_CONTINUE_HOLD_MS 500u // 长按约 0.5s 触发连发
///////////////////////////////
/* 右发射轮电机PID参数 */
#define PID_SHOOT_RIGHT_MOTOR_KP 3.0f         // 比例系数
#define PID_SHOOT_RIGHT_MOTOR_KI 0.02f        // 积分系数
#define PID_SHOOT_RIGHT_MOTOR_KD 0.0f         // 微分系数
#define PID_SHOOT_RIGHT_MOTOR_MAXOUT 10000.0f // PID最大输出
#define PID_SHOOT_RIGHT_MOTOR_MAXI 9000.0f    // PID最大积分输出

/* 左发射轮电机PID参数 */
#define PID_SHOOT_LEFT_MOTOR_KP 3.0f        // 比例系数
#define PID_SHOOT_LEFT_MOTOR_KI 0.02f       // 积分系数
#define PID_SHOOT_LEFT_MOTOR_KD 0.0f        // 微分系数
#define PID_SHOOT_LEFT_MOTOR_MAXOUT 10000.0 // PID最大输出
#define PID_SHOOT_LEFT_MOTOR_MAXI 9000.0f   // PID最大积分输出

/* 发射轮锁定PID参数 */
#define PID_SHOOT_LOCK_KP 1000.0f // 锁定时的比例系数
/////////////////////////////////
/* 拨弹轮电机PID参数 */
#define PID_SHOOT_PULL_MOTOR_KP     400.0f   // 比例系数
#define PID_SHOOT_PULL_MOTOR_KI     0.0f     // 积分系数
#define PID_SHOOT_PULL_MOTOR_KD     0.0f     // 微分系数
#define PID_SHOOT_PULL_MOTOR_MAXOUT 10000.0f // PID最大输出
#define PID_SHOOT_PULL_MOTOR_MAXI   7000.0f  // PID最大积分输出
////////////////////////////////////
// 电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
// 电机编码值转化成角度值
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192

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

/* ROLL轴速度环PID参数 */
#define ROLL_SPEED_PID_KP       2.66f  // 比例系数
#define ROLL_SPEED_PID_KI       0.005f // 积分系数
#define ROLL_SPEED_PID_KD       0.0f   // 微分系数
#define ROLL_SPEED_PID_MAX_OUT  10.0f   // PID最大输出
#define ROLL_SPEED_PID_MAX_IOUT 0.5f   // PID最大积分输出

/* ROLL轴角度环PID参数（基于陀螺仪解算角度） */
#define ROLL_GYRO_ABSOLUTE_PID_KP       10.0f // 比例系数
#define ROLL_GYRO_ABSOLUTE_PID_KI       0.1f  // 积分系数
#define ROLL_GYRO_ABSOLUTE_PID_KD       0.0f  // 微分系数
#define ROLL_GYRO_ABSOLUTE_PID_MAX_OUT  15.0f // PID最大输出
#define ROLL_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f  // PID最大积分输出
#ifdef Gimbal_Roll

#endif
typedef enum
{
    SHOOT_STOP = 0,        // 停止射击，摩擦轮停止转动
    SHOOT_READY_FRIC,      // 摩擦轮启动，直到达到指定转速，自动进入SHOOT_READY
    SHOOT_READY,           // 摩擦轮热身完毕
    SHOOT_BULLET,          // 射击
    SHOOT_CONTINUE_BULLET, // 持续射击
} shoot_mode_e;
/**
 * @brief 云台发射系统结构体
 * @details 包含发射轮、拨弹轮的控制参数和状态信息
 */
typedef struct
{
    dji_motor_measure_t *pull_motor;        // 拨弹轮电机测量数据指针
    dji_motor_measure_t *shoot_motor_left;  // 左发射轮电机测量数据指针
    dji_motor_measure_t *shoot_motor_right; // 右发射轮电机测量数据指针
    float shoot_left_rad;                   // 左发射轮弧度位置
    float shoot_right_rad;                  // 右发射轮弧度位置
    float shoot_pull_rad;                   // 拨弹轮弧度位置

    float shoot_speed;       // 发射轮当前转速
    float pull_target_speed; // 拨弹轮目标转速（由 trigger_motor_turn_back 写入）

    PidTypeDef Pull_PID;        // 拨弹轮PID控制器
    PidTypeDef Shoot_right_PID; // 右发射轮PID控制器
    PidTypeDef Shoot_left_PID;  // 左发射轮PID控制器
    float shoot_target_speed;   // 发射轮目标转速

    shoot_mode_e shoot_mode; // 射击状态机（集中所有射击状态转换）
    bool_t bullet_done;      // 单发完成标志（shoot_bullet_control 置位，状态机消费）
    uint16_t block_count;    // 堵转持续计数（每 2ms +1，>= BLOCK_TIME_COUNT 判定卡弹）
    uint16_t reverse_count;  // 反转持续计数（每 2ms +1，>= REVERSE_TIME_COUNT 复位）
} gimbal_shoot_t;

/**
 * @brief 云台位置控制结构体
 * @details 包含YAW和PITCH轴的位置控制、PID参数、滤波器等
 */
typedef struct
{
    dm_motor_measure_t *yaw_motor_measure;    // YAW轴电机测量数据指针
    dm_motor_measure_t *pitch_motor_measure; // PITCH轴电机测量数据指针
    dm_motor_measure_t *roll_motor_measure; // ROLL轴电机测量数据指针
    const Usb_AutoAim_t *usb_autoAim_ptr;     // USB自瞄数据指针

    float pitch_absolute_pos; // PITCH轴绝对位置
    float yaw_absolute_pos;   // YAW轴绝对位置
    float roll_absolute_pos;   // ROLL轴绝对位置

    float pitch_relatiive_pos; // PITCH轴相对位置
    float yaw_relattive_pos;   // YAW轴相对位置
    float roll_relattive_pos;   // ROLL轴相对位置

    float pitch_target_pos;      // PITCH轴目标位置
    float last_pitch_target_pos; // PITCH轴上次目标位置
    float yaw_target_pos;        // YAW轴目标位置
    float last_yaw_target_pos;   // YAW轴上次目标位置

    float roll_target_pos;        // ROLL轴目标位置
    float last_roll_target_pos;   // ROLL轴上次目标位置

    float init_pitch_pos; // PITCH轴初始位置
    float init_yaw_pos;   // YAW轴初始位置
    float init_roll_pos;   // ROLL轴初始位置

    PidTypeDef gimbal_motor_gyro_pid;           // PITCH轴速度环PID控制器
    PidTypeDef gimbal_motor_absolute_angle_pid; // PITCH轴角度环PID控制器

    PidTypeDef yaw_motor_gyro_pid;     // YAW轴速度环PID控制器
    PidTypeDef yaw_absolute_angle_pid; // YAW轴角度环PID控制器

    PidTypeDef roll_motor_gyro_pid;     // ROLL轴速度环PID控制器
    PidTypeDef roll_absolute_angle_pid; // ROLL轴角度环PID控制器

    first_order_filter_type_t filter_rc_pitch_vel_set; // 遥控PITCH速度设定滤波器
    first_order_filter_type_t filter_rc_yaw_vel_set;   // 遥控YAW速度设定滤波器
    first_order_filter_type_t filter_rc_roll_vel_set;   // 遥控ROLL速度设定滤波器

    first_order_filter_type_t filter_MiniPc_pitch_set; // 小电脑PITCH设定滤波器
    first_order_filter_type_t filter_MiniPc_yaw_set;   // 小电脑YAW设定滤波器
    first_order_filter_type_t filter_MiniPc_roll_set;   // 小电脑ROLL设定滤波器

    bool_t calibrate_warning; // 校准警告标志

    float add_yaw;   // YAW轴附加角度
    float add_pitch; // PITCH轴附加角度
    float add_roll; // ROLL轴附加角度
    float tmp;       // 临时变量
} gimbal_pos_control_t;

/**
 * @brief 云台工作模式枚举
 * @details 定义云台的各种工作状态和控制模式
 */
typedef enum {
    GIMBAL_MOD_NO_FORCE  = 0, // 无力模式，电机无输出，默认模式
    GIMBAL_MOD_SLOW_CALI = 1, // 慢速校准模式，用于初始化校准
    GIMBAL_MOD_NORMAL    = 2, // 正常控制模式，手动遥控
    GIMBAL_MOD_AutoAim   = 3, // 自动瞄准模式，视觉辅助

    GIMBAL_MOD_SHAKING       = 4, // 小陀螺模式，云台摇摆躲避
    GIMBAL_MOD_CONTROL_BY_PC = 5, // PC控制模式，上位机控制
    GIMBAL_MOD_AutoBalance  = 6,    //ROLL轴自稳模式
} gimbal_mod_e;

/**
 * @brief 云台控制总结构体
 * @details 包含云台位置控制、发射系统、遥控信息、工作模式等所有相关数据
 */
typedef struct
{
    gimbal_pos_control_t gimbal_pos; // 云台位置控制结构体
    gimbal_shoot_t gimbal_shoot;     // 云台发射系统结构体
    #ifdef USE_SBUS_PROTOCOL
    const SBUS_Remote_Info_Typedef *gimbal_RC;// SBUS遥控器信息指针
    #else
    const Remote_Info_Typedef *gimbal_RC; // DBUS遥控器信息指针
    #endif

    gimbal_mod_e last_gimbal_mod; // 上一次工作模式
    gimbal_mod_e gimbal_mod;      // 当前工作模式

    const Usb_dpkg_data_t *minipc_info; // 小电脑通信数据指针
    const INS_Info_Typedef *ins_info;   // 惯性导航系统信息指针
} gimbal_t;

/* 全局变量声明 */
// extern gimbal_t gimbal;

/**
 * @brief 云台控制任务主函数
 * @param argument FreeRTOS任务参数
 */
void Gimbal_Task(void const *argument);
const gimbal_t* get_gimbal_point(void);
#endif