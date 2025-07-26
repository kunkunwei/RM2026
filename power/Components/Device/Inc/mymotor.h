#ifndef __MYMOTOR_H__
#define __MYMOTOR_H__

#include "main.h"

/* CAN send and receive ID */
//GM6020    电流给定值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A。转矩常熟 741 mN·m/A
typedef enum
{
    CAN1_SHOOT_MOTOR_LEFT_ID = 0x205,
    CAN1_SHOOT_MOTOR_RIGHT_ID = 0x206,
    CAN1_SHOOT_PULL_MOTOR_ID = 0x207,
    
    CAN1_CMD_ALL_ID = 0x1FF,
    CAN1_PITCH_MOTOR_ID = 0x208,
    CAN1_POWER_ID = 0x209,
    
    CAN2_WHEEL_MOTRO_RIGHT_FRONT_ID = 0x201,
    CAN2_WHEEL_MOTRO_LEFT_FRONT_ID = 0x202,
    CAN2_WHEEL_MOTRO_LEFT_BACK_ID = 0x203,
    CAN2_WHEEL_MOTRO_RIGHT_BACK_ID = 0x204,
    CAN2_CMD_ALL_ID = 0x200,

    CAN2_YAW_MOTOR_ID = 0x01

} can_msg_id_e;

typedef struct
{
    float voltage;       //电压
    float current;       //电流
    float power;         //功率
}Chassis_power_measure_t;

typedef struct 
{   
    uint16_t pos;
    int16_t rpm;
    int16_t current;
    uint8_t temp;

    float real_w;
    float real_pos;

    float speed;
    float accel;
    
    int16_t target_current;
}dji_motor_measure_t;
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_pitch_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);

extern dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;//2摩擦轮，1拨蛋
extern dji_motor_measure_t chassis_motor[4];//四个轮子
extern dji_motor_measure_t pitch_motor;

dji_motor_measure_t* get_shoot_motor_left();
dji_motor_measure_t* get_shoot_motor_right();
dji_motor_measure_t* get_shoot_motor_pull();
dji_motor_measure_t* get_chassis_motor(uint8_t i);

dji_motor_measure_t* get_pitch_motor();

/////////////////////////////////////////////////////////////////////////////////
//dm8009电机统一数据结构体
typedef struct
{
    uint8_t err;           //err
    float pos;           //位置
    float speed;         //速度
    float tor;           //转矩
    int8_t tmos_tmper;     //tmos温度
    int8_t coil_tmper;     //线圈温度

    float target_tor;
}dm_motor_measure_t;

//lk9025电机统一数据结构体
typedef struct
{
    int16_t speed;                //速度
    int16_t tor_current;          //转矩电流
    uint16_t pos;                 //编码器位置
    int8_t  tmper;                //电机温度
} lk9025_motor_measure_t;

// void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, uint8_t *rx_message);
// void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, uint8_t *rx_message);
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message);

extern dm_motor_measure_t yaw_motor;

dm_motor_measure_t* get_yaw_motor();
// // lk9025_motor_measure_t motor_right, motor_left;
// // static dm8009_motor_measure_t motor_joint[4];
// //返回右驱动轮电机变量地址，通过指针方式获取原始数据
// extern const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
// //返回左驱动轮电机变量地址，通过指针方式获取原始数据
// extern const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
// //返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
// extern const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);
extern dji_motor_measure_t feiniao_yaw,feibiao_roll,feibiao_pull;
Chassis_power_measure_t* get_chassis_power_measure();
dji_motor_measure_t* get_feibiao_yaw_motor();
dji_motor_measure_t* get_feibiao_rollmotor();
dji_motor_measure_t* get_feibiao_pull_motor();
#endif // !__MYMOTOR_H__