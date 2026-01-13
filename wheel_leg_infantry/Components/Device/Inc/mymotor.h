#ifndef __MYMOTOR_H__
#define __MYMOTOR_H__

#include "main.h"

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x3FE,
    CAN_dm8009_M1_ID = 0x01,
    CAN_dm8009_M2_ID = 0x02,
    CAN_dm8009_M3_ID = 0x03,
    CAN_dm8009_M4_ID = 0x04,

    CAN1_Chassis_ID=0x60,         // 底盘CAN1发送反馈帧ID,反馈底盘数据
    CAN1_Chassis_ID_1=0x60,         // CAN1发送命令帧1ID,控制底盘
    CAN1_Chassis_ID_2=0x61,         // CAN1发送命令帧2ID,控制底盘
    CAN_RIGHT_MOTOR_ID = 0x141,
    CAN_LEFT_MOTOR_ID = 0x142,
    CAN_WHEEL_ALL_ID = 0x280,
} can_msg_id_e;

//dm8009电机统一数据结构体
typedef struct
{
    uint8_t err;           //err
    float pos;           //位置
    float speed;         //速度
    float tor;           //转矩
    int8_t tmos_tmper;     //tmos温度
    int8_t coil_tmper;     //线圈温度
}dm8009_motor_measure_t;

//lk9025电机统一数据结构体
typedef struct
{
    int16_t speed;                //速度
    int16_t tor_current;          //转矩电流
    uint16_t pos;                 //编码器位置
    int8_t  tmper;                //电机温度
} lk9025_motor_measure_t;

void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, uint8_t *rx_message);
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, uint8_t *rx_message);

// lk9025_motor_measure_t motor_right, motor_left;
// static dm8009_motor_measure_t motor_joint[4];
//返回右驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
//返回左驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
//返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);


#endif // !__MYMOTOR_H__