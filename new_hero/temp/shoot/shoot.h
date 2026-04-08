/**
  ****************************RM Warrior 2023****************************
  * @file       shoot.c/h
  * @brief      射击功能.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2023/2/         pxx              ......
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************RM Warrior 2023****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "main.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"


//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       4
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400
//遥控器射击开关打下档一段时间后 连续发射子弹 
#define RC_S_LONG_TIME              200
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f       // 2PI/60/19
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f   // PI / (8192*180)
#define FULL_COUNT                  18  //  36/2
//m3508转化成摩擦轮速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.0001635973f  // 2PI/60/(3591/187) * (0.06/2)
//拨弹速度
#define LOW_TRIGGER_SPEED           13.0f
#define HIGH_TRIGGER_SPEED          15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
// #define SWITCH_TRIGGER_ON           0       //子弹已到达限位开关
// #define SWITCH_TRIGGER_OFF          1       //子弹未到达限位开关

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                600
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        280.0f
#define TRIGGER_ANGLE_PID_KI        1.0f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f
//摩擦轮电机
#define Friction_PID_KP    3.0f
#define Friction_PID_KI    0.02f
#define Friction_PID_KD    0.0f

#define Friction_PID_MAX_OUT      10000.0f
#define Friction_PID_MAX_IOUT     9000.0f

#define SHOOT_HEAT_REMAIN_VALUE     10

typedef enum
{
    SHOOT_STOP = 0,         //停止射击，摩擦轮停止转动
    SHOOT_READY_FRIC,       //摩擦轮启动，直到达到指定转速，软件自动进入SHOOT_READY
    SHOOT_READY,            //摩擦轮热身完毕 
    SHOOT_BULLET,           //射击
    SHOOT_CONTINUE_BULLET,  //持续射击
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    PidTypeDef trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;//拨弹电机速度
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;//电机轴转动圈数

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    uint16_t heat_limit;
    uint16_t heat;

    bool_t high_speed_on;
} shoot_control_t;

typedef struct
{
	PidTypeDef fric_motor_pid;
	const motor_measure_t *fric_motor_measure;
	int16_t given_current;
	fp32 speed;//摩擦轮电机速度
    fp32 speed_set;
} friction_control_t;
//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern int16_t CAN2_OK;
extern int16_t CAN1_OK;
extern void shoot_init(void);
extern int16_t shoot_control_loop(void);
extern friction_control_t fric1_M_control,fric2_M_control; 
const shoot_control_t *get_shoot_control_point(void);
extern const friction_control_t *get_shoot_fric1_control_point(void);
extern const friction_control_t *get_shoot_fric2_control_point(void);

#endif

