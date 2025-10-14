#ifndef __GIMBAL_TASK__
#define __GIMBAL_TASK__

#include "main.h"
#include "can_task.h"
#include "old_pid.h"
#include "chassis_behaviour.h"
#include "user_lib.h"
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
#define PID_PITCH_MOTOR_VEL_KP         2000.0f
#define PID_PITCH_MOTOR_VEL_KI         0.01f
#define PID_PITCH_MOTOR_VEL_KD         0.0f
#define PID_PITCH_MOTOR_VEL_MAXOUT     12000.0f
#define PID_PITCH_MOTOR_VEL_MAXI       200.0f
////////////////////////////////////
/////////////////////////////////
#define PID_PITCH_MOTOR_POS_KP         16888.0f
#define PID_PITCH_MOTOR_POS_KI         10.0f
#define PID_PITCH_MOTOR_POS_KD         6.66f
#define PID_PITCH_MOTOR_POS_MAXOUT     15000.0f
#define PID_PITCH_MOTOR_POS_MAXI       2800.0f
////////////////////////////////////

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//选择用户自定义模式状态 开关通道号
// #define SUPER_MODE_CHANNEL 1
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 0
#define PitchChannel 1
#define ModeChannel 0
//测试按键
#define TestKeyBoard KEY_PRESSED_OFFSET_G
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.000004f
#define Pitch_RC_SEN -0.000006f //0.005
//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen 0.000025f
#define Pitch_Mouse_Sen 0.00013f
//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f
//云台控制周期
#define GIMBAL_CONTROL_TIME_MS 1
#define GIMBAL_CONTROL_TIME 0.001f

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN 0
#define YAW_TURN 1
#define TRIGGER_TURN 1

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.05f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.003f
#define GIMBAL_INIT_YAW_SPEED   0.006f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f
//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000
//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

#define YAW_ECD_TO_RAD Motor_Ecd_to_Rad/3

#define GIMBAL_ACCEL_YAW_NUM 0.02f
#define GIMBAL_ACCEL_PITCH_NUM 0.05f
typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} Gimbal_Motor_Mode_e;
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;     //云台电机结构体
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;    //绝对角度位置pid
    Gimbal_PID_t gimbal_motor_relative_angle_pid;    //相对角度位置pid
    PidTypeDef gimbal_motor_gyro_pid;                //yaw电机 速度电流pid
    Gimbal_Motor_Mode_e gimbal_motor_mode;           //云台控制状态机
    Gimbal_Motor_Mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;     //yaw电机转子中间位置
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    first_order_filter_type_t gimbal_cmd_slow_set;    //一阶低通滤波上位机期望

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const Remote_Info_Typedef *gimbal_rc_ctrl;    //遥控器结构体指针
    const ROS_Msg_t *gimbal_ros_msg;    //上位机指令指针
    const fp32 *gimbal_INT_angle_point; //陀螺仪数据指针
    const fp32 *gimbal_INT_gyro_point;  //加速度计数据指针
    Gimbal_Motor_t gimbal_yaw_motor;    //云台yaw电机结构体
    Gimbal_Motor_t gimbal_pitch_motor;  //云台pitch电机结构体
    Gimbal_Cali_t gimbal_cali;          //校准结果结构体
    int8_t ecd_count;                   //编码计数
    int8_t last_super_channel;          //上一次遥控器开关所在的位置
} Gimbal_Control_t;

typedef struct 
{   
    dji_motor_measure_t* pull_motor;
    dji_motor_measure_t* shoot_motor_left;
    dji_motor_measure_t* shoot_motor_right;

    float shoot_speed;

    PidTypeDef Pull_PID;
    PidTypeDef Shoot_right_PID;
    PidTypeDef Shoot_left_PID;

    bool_t shoot_ready_flag;
}Gimbal_Shoot_t;

typedef struct
{
    dm_motor_measure_t* yaw_motor;
    dji_motor_measure_t* pitch_motor;

    float pitch_rad;
    float pitch_offset;

    float yaw_rad;
    float yaw_offset;

    PidTypeDef Pitch_Vel_PID;
    PidTypeDef Pitch_Pos_PID;

    PidTypeDef Yaw_PID;

    bool_t calibrate_warning;
}gimbal_pos_control_t;

typedef struct
{
    gimbal_pos_control_t gimbal_pos;
    Gimbal_Shoot_t gimbal_shoot;

    // const control_mode_t* rc_control_mode;
    // const Remote_Info_Typedef *gimbal_RC;
}gimbal_t;

// extern gimbal_t gimbal;
void Gimbal_Task(void const * argument);

#endif 