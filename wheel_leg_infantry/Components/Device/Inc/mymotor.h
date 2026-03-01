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
/**
 * @brief DJI电机测量数据结构体
 * @details 用于存储DJI电机的反馈数据和计算得到的物理量
 */
typedef struct
{
    uint16_t pos;          // 编码器位置原始值(0-8191)
    int16_t rpm;           // 转速原始值(rpm)
    int16_t current;       // 电流原始值(mA)
    uint8_t temp;          // 温度(°C)

    float real_w;          // 实际角速度(rad/s)
    float real_pos;        // 实际位置(rad)

    float speed;           // 目标速度
    float accel;           // 加速度

    int16_t target_current; // 目标电流值
}dji_motor_measure_t;

void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, uint8_t *rx_message);
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, uint8_t *rx_message);
/* DJI电机数据解析函数声明 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);

// lk9025_motor_measure_t motor_right, motor_left;
// static dm8009_motor_measure_t motor_joint[4];
extern dji_motor_measure_t yaw_motor;  // Yaw轴GM6020电机
//返回右驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void);
//返回左驱动轮电机变量地址，通过指针方式获取原始数据
extern const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void);
//返回关节电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i);
/* DJI电机访问函数声明 */

dji_motor_measure_t* get_yaw_motor();  // 获取Yaw轴电机指针

#endif // !__MYMOTOR_H__