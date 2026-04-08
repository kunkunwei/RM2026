/**
 * @file Gimbal_task.h
 * @brief 云台控制任务头文件
 * @details 包含云台PID控制、发射系统控制、模式切换等功能的定义
 * @author RM Team
 * @date 2025
 */

#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

#include "config.h"
#include "mymotor.h"
#include "can_task.h"
#include "old_pid.h"
// #include "chassis_behaviour.h"

#include "pc_uart_ctrl.h"
#include "user_lib.h"
#include "usb.h"

/////////////////////
/* 底盘类型选择宏定义 */

/* 控制方式选择宏定义 */
// #define USE_RC_CONTROL // 使用遥控器控制模式
#define USE_PC_CONTROL    // 使用图传链路遥控器控制模式
#define USE_PC_RM_CONTROL // 使用图传链路电脑选手端控制模式
#define REDDUCE_K 1.0f    // YAW减速系数
//////////////////////

//////////////////////////

#ifdef USE_SBUS_PROTOCOL
/* 遥控器开关定义 */
#define LEFT_1_SWITCH 0  // 左1拨杆
#define LEFT_2_SWITCH 1  // 左2拨杆
#define RIGHT_1_SWITCH 3 // 右1拨杆
#define RIGHT_2_SWITCH 2 // 右2拨杆
/* 遥控器通道定义 */
#define RC_RIGHT_X_CH 0 // 右摇杆X轴通道
#define RC_RIGHT_Y_CH 1 // 右摇杆Y轴通道
#define RC_LEFT_X_CH 3  // 左摇杆X轴通道
#define RC_LEFT_Y_CH 2  // 左摇杆Y轴通道
#define RC_ROLL_1_CH 4  // 滚转通道1
#define RC_ROLL_2_CH 5  // 滚转通道2
///////////////////////
/* 云台遥控灵敏度设置 */
#define GIMBAL_WZ_RC_SEN 0.000006f     // YAW轴遥控灵敏度
#define GIMBAL_PITCH_RC_SEN -0.000003f // PITCH轴遥控灵敏度
#define GIMBAL_ROLL_RC_SEN -0.000003f  // ROLL轴遥控灵敏度
#else
/* 遥控器开关定义 */
#define RIGHT_SWITCH 0  // 右拨杆
#define LEFT_SWITCH 1   // 左拨杆
/* 遥控器通道定义 */
#define RC_RIGHT_X_CH 0 // 右摇杆X轴通道
#define RC_RIGHT_Y_CH 1 // 右摇杆Y轴通道
// #define RC_LEFT_X_CH 2  // 左摇杆X轴通道
// #define RC_LEFT_Y_CH 3  // 左摇杆Y轴通道
/*图传通道*/
#define RC_LEFT_X_CH 3  // 左摇杆X轴通道
#define RC_LEFT_Y_CH 2  // 左摇杆Y轴通道
#define RC_ROLL_CH 4    // 滚转通道
/*滚转通道数值判断界限*/
#define CH4_HIGH_VALUE 600
#define CH4_LOW_VALUE 500
///////////////////////
/* 云台遥控灵敏度设置 */
#define GIMBAL_WZ_RC_SEN -0.000006f   // YAW轴遥控灵敏度
#define GIMBAL_PITCH_RC_SEN 0.000007f // PITCH轴遥控灵敏度

#define GIMBAL_WZ_MOUSE_SEN -0.000006f   // YAW轴遥控灵敏度
#define GIMBAL_PITCH_MOUSE_SEN 0.000007f // PITCH轴遥控灵敏度
#define GIMBAL_ROLL_MOUSE_SEN 0.000007f  // ROLL轴遥控灵敏度

#endif

/* PITCH轴机械限位（基于电机编码器角度，零点定义为枪管水平） */
#define MAX_PITCH_RAD 0.22f  // 编码器正向最大机械角度（rad）
#define MIN_PITCH_RAD -0.30f // 编码器负向最大机械角度（rad）

///////////////////////////////
/* 发射系统参数设置 */
#define SHOOT_SPD 350.0f                    // 发射轮目标转速，750对应22-25m/s
#define SHOOT_CHANNEL_TO_SPEED 1.0 / 660.0f // 通道值转换为速度的系数（660->1rad）

//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00545324260857041989782339471524f       // 2PI/60/(3591/187)
#define MOTOR_RPM_TO_SPEED_FRIC     0.10471975511965977461542144610932       // 2PI/60
#define MOTOR_ECD_TO_ANGLE          0.000019970370880995190055505595881022f   // PI / (8192*3591/187)
#define FULL_COUNT                  1975                                      // 3591/2

/* 拨弹轮参数：角度用于判定一发，速度用于控制弹频 */
#define SHOOT_SINGLE_BULLET_RAD  PI/3  // 单发拨弹轮累计转角阈值（π/3，60度）
#define SHOOT_PULL_FORWARD_SPEED -10.0f      // 拨弹轮正向目标角速度(rad/s)，调大可提升弹频（负号表示电机倒置安装）
#define SHOOT_PULL_REVERSE_SPEED 5.0f     // 堵转解卡反向角速度(rad/s)（正号表示电机倒置安装）
#define SHOOT_SINGLE_TIMEOUT_MS 900u        // 单发超时兜底，超时后强制认为一发完成
#define PI_THIRD 1.0471975511965976f        // π/3，60度，拨弹盘6个凹槽
#define PI_SIXTH 0.5235987755982988f        // π/6，30度，用于额外补偿角度

/* 射击间隔控制参数 */
#define SHOOT_INTERVAL_MS 500u              // 射击最小间隔时间（500ms）
#define SHOOT_DEBOUNCE_MS 50u               // 射击按键消抖时间

/* 拨弹轮堵转检测参数（计数基于 2ms 控制周期）*/
#define BLOCK_TRIGGER_SPEED 1.0f // rad/s，拨弹轮速度低于此值视为可能堵转
#define BLOCK_TIME_COUNT 250u    // 250×2ms = 500ms 持续慢速后判定卡弹
#define REVERSE_TIME_COUNT 300u  // 300×2ms = 600ms 反转持续时间后自动复位
/* 连发长按阈值（基于 osKernelSysTick 计数，默认 1tick=1ms）*/
#define SHOOT_CONTINUE_HOLD_MS 900u // 长按约 0.5s 触发连发
///////////////////////////////
/* 发射轮电机PID参数 */
#define PID_FRIC_MOTOR_KP 3.0f         // 比例系数
#define PID_FRIC_MOTOR_KI 0.02f        // 积分系数
#define PID_FRIC_MOTOR_KD 0.0f         // 微分系数
#define PID_FRIC_MOTOR_MAXOUT 10000.0f // PID最大输出
#define PID_FRIC_MOTOR_MAXI 9000.0f    // PID最大积分输出



/* 发射轮锁定PID参数 */
#define PID_SHOOT_LOCK_KP 800.0f // 锁定时的比例系数
/////////////////////////////////
/* 拨弹轮电机PID参数 */
#define PID_PULL_MOTOR_KP  3800.0f   // 比例系数
#define PID_PULL_MOTOR_KI  10.0f   // 积分系数
#define PID_PULL_MOTOR_KD  3.0f   // 微分系数
#define PID_PULL_MOTOR_MAXOUT 15000.0f // PID最大输出
#define PID_PULL_MOTOR_MAXI 2000.0f    // PID最大积分输出

////////////////////////////////////
// 电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
// 电机编码值转化成角度值
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
////////////////////////////////////

/* PITCH轴速度环PID参数 */
#define PITCH_SPEED_PID_KP 0.66f      // 比例系数
#define PITCH_SPEED_PID_KI 0.000f     // 积分系数
#define PITCH_SPEED_PID_KD 0.005f     // 微分系数
#define PITCH_SPEED_PID_MAX_OUT 5.0f  // PID最大输出
#define PITCH_SPEED_PID_MAX_IOUT 0.5f // PID最大积分输出

/* PITCH轴角度环PID参数（基于陀螺仪解算角度） */
#define PITCH_GYRO_ABSOLUTE_PID_KP 20.0f      // 比例系数
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.1f       // 积分系数
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.5f       // 微分系数
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 5.0f  // PID最大输出
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 1.0f // PID最大积分输出

/* YAW轴速度环PID参数 */
#define YAW_SPEED_PID_KP 1.0f       // 比例系数
#define YAW_SPEED_PID_KI 0.0f          // 积分系数
#define YAW_SPEED_PID_KD 0.002f          // 微分系数
#define YAW_SPEED_PID_MAX_OUT 10.0f // PID最大输出
#define YAW_SPEED_PID_MAX_IOUT 2.0f // PID最大积分输出

/* YAW轴相对角度角度环PID参数 */
#define YAW_GYRO_RELATIVE_PID_KP 10.0f      // 比例系数
#define YAW_GYRO_RELATIVE_PID_KI 0.05f      // 积分系数
#define YAW_GYRO_RELATIVE_PID_KD 0.1f       // 微分系数
#define YAW_GYRO_RELATIVE_PID_MAX_OUT 10.0f // PID最大输出
#define YAW_GYRO_RELATIVE_PID_MAX_IOUT 1.2f // PID最大积分输出

/* YAW轴角度环PID参数（基于陀螺仪解算角度） */
#define YAW_GYRO_ABSOLUTE_PID_KP 10.0f      // 比例系数
#define YAW_GYRO_ABSOLUTE_PID_KI 0.05f      // 积分系数
#define YAW_GYRO_ABSOLUTE_PID_KD 0.1f       // 微分系数
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f // PID最大输出
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 1.2f // PID最大积分输出

/* 前馈控制参数 */
#define PITCH_VELOCITY_FF_GAIN 0.05f        // PITCH轴速度前馈增益（减小）
#define PITCH_ACCELERATION_FF_GAIN 0.005f   // PITCH轴加速度前馈增益（减小）
#define YAW_VELOCITY_FF_GAIN 0.001f         // YAW轴速度前馈增益（大幅减小）
#define YAW_ACCELERATION_FF_GAIN 0.0001f    // YAW轴加速度前馈增益（大幅减小）
#define YAW_DISTURBANCE_FF_GAIN 0.5f        // YAW轴干扰前馈增益（减小）
#define YAW_FOLLOW_FF_GAIN 0.1f             // YAW轴跟随模式干扰前馈增益（减小）

typedef enum
{
    SHOOT_STOP = 0,        // 停止射击，摩擦轮停止转动
    SHOOT_READY_FRIC,      // 摩擦轮启动，直到达到指定转速，自动进入SHOOT_READY
    SHOOT_READY,           // 摩擦轮热身完毕
    SHOOT_BULLET         // 射击
} shoot_mode_e;
/**
 * @brief 云台发射系统结构体
 * @details 包含发射轮、拨弹轮的控制参数和状态信息
 */
typedef struct
{
    dji_motor_measure_t *pull_motor;        // 拨弹轮电机测量数据指针
    dji_motor_measure_t *Fric_motor[4];  // 摩擦轮电机测量数据指针
    float Fric_rad[4];                   // 摩擦轮角速度

    float pull_rad;                   // 拨弹轮角速度
    float set_angle;    // 拨弹轮目标角度（单发模式使用）
    float pull_abs_pos;                   // 拨弹轮角度
    // float block_time;                   // 堵转持续时间

    int16_t ecd_count;//电机轴转动圈数
    float pull_target_speed; // 拨弹轮目标转速（由 trigger_motor_turn_back 写入）

    PidTypeDef Pull_PID;        // 拨弹轮PID控制器
    PidTypeDef Fric_PID[4]; // 摩擦轮PID控制器

    float Fric_target_speed;   // 发射轮目标转速

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
    dm_motor_measure_t *yaw_motor_measure;  // YAW轴电机测量数据指针
    dm_motor_measure_t *pitch_motor_measure; // PITCH轴电机测量数据指针
    const Usb_AutoAim_t *usb_autoAim_ptr;    // USB自瞄数据指针

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

    PidTypeDef gimbal_motor_gyro_pid;           // PITCH轴速度环PID控制器
    PidTypeDef gimbal_motor_absolute_angle_pid; // PITCH轴角度环PID控制器

    PidTypeDef yaw_motor_gyro_pid;     // YAW轴速度环PID控制器
    PidTypeDef yaw_absolute_angle_pid; // YAW轴角度环PID控制器
    PidTypeDef yaw_relative_angle_pid; // YAW轴相对角度环PID控制器

    first_order_filter_type_t filter_rc_pitch_vel_set; // 遥控PITCH速度设定滤波器
    first_order_filter_type_t filter_rc_yaw_vel_set;   // 遥控YAW速度设定滤波器

    first_order_filter_type_t filter_MiniPc_pitch_set; // 小电脑PITCH设定滤波器
    first_order_filter_type_t filter_MiniPc_yaw_set;   // 小电脑YAW设定滤波器

    bool_t calibrate_warning; // 校准警告标志

    float add_yaw;   // YAW轴附加角度
    float add_pitch; // PITCH轴附加角度
    float tmp;       // 临时变量
} gimbal_pos_control_t;

/**
 * @brief 云台工作模式枚举
 * @details 定义云台的各种工作状态和控制模式
 */
typedef enum
{
    GIMBAL_MOD_NO_FORCE = 0,  // 无力模式，电机无输出，默认模式
    GIMBAL_MOD_SLOW_CALI = 1, // 慢速校准模式，用于初始化校准
    GIMBAL_MOD_NORMAL = 2,    // 正常控制模式，手动遥控，底盘跟随云台模式，底盘保持与云台相对位置不变，底盘的yaw轴无法被遥控器控制
    GIMBAL_MOD_AutoAim = 3,   // 自动瞄准模式，视觉辅助
    GIMBAL_MOD_FLOW_CHASSIS = 5,  // 云台跟随底盘模式，当底盘转动时，云台保持相对底盘位置不变，此时云台无法被遥控器控制
} gimbal_mod_e;




/**
 * @brief 云台控制总结构体
 * @details 包含云台位置控制、发射系统、遥控信息、工作模式等所有相关数据
 */
typedef struct
{
    gimbal_pos_control_t gimbal_pos; // 云台位置控制结构体
    gimbal_shoot_t gimbal_shoot;     // 云台发射系统结构体

#ifdef USE_PC_CONTROL
    const PC_Ctrl_Info_t *VT03_ctrl;					// 图传遥控器指针

#else
#ifdef USE_SBUS_PROTOCOL
    const SBUS_Remote_Info_Typedef *gimbal_RC; // SBUS遥控器信息指针
#else
    const Remote_Info_Typedef *gimbal_RC; // DBUS遥控器信息指针
#endif
#endif
    gimbal_mod_e last_gimbal_mod; // 上一次工作模式
    gimbal_mod_e gimbal_mod;      // 当前工作模式
    bool VT03_fn1_last;                   // 上一次图传链路fn_1按键状态
    bool VT03_fn2_last;                   // 上一次图传链路fn_2按键状态
    bool VT03_pause_last;                 // 上一次图传链路pause按键状态
    bool auto_aim_mode;                   // 自动瞄准模式
    bool spinning_mode;					  // 小陀螺模式
    bool last_spinning_mode;					  // 上次小陀螺模式



    const Usb_AutoAim_t *minipc_info; // 小电脑通信数据指针
    const INS_Info_Typedef *ins_info;   // 惯性导航系统信息指针
} gimbal_t;

/* gimbal 已在 Gimbal_task.c 中声明为 static，外部通过下方接口只读访问 */

/**
 * @brief 获取云台控制结构体只读指针
 * @return 指向内部 static gimbal 结构体的只读指针
 */
const gimbal_t *get_gimbal_point(void);

/**
 * @brief 查询云台初始化是否已完成
 * @return true 表示 init_gimbal_control() 已执行完毕，指针均有效
 */
bool is_gimbal_init_done(void);

/**
 * @brief 云台控制任务主函数
 * @param argument FreeRTOS任务参数
 */
void Gimbal_Task(void const *argument);

#endif
