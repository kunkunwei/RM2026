#ifndef __MYMOTOR_H__
#define __MYMOTOR_H__

#include "main.h"

/* CAN发送和接收ID */
//GM6020    电流给定值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A。转矩常数 741 mN·m/A
typedef enum
{
    CAN2_SHOOT_MOTOR_LEFT_ID = 0x205,    // CAN2 左摩擦轮电机ID
    CAN2_SHOOT_MOTOR_RIGHT_ID = 0x206,   // CAN2 右摩擦轮电机ID
    CAN2_SHOOT_PULL_MOTOR_ID = 0x207,    // CAN2 拨蛋电机ID
    // CAN2_PITCH_MOTOR_ID = 0x208,

    CAN2_CMD_ALL_ID = 0x1FF,             // CAN2 所有电机命令ID

    CAN1_CMD_ALL_ID = 0x200,             // CAN1 所有电机命令ID

    CAN1_YAW_MOTOR_ID = 0x00,            // CAN1 云台电机ID
    CAN2_PITCH_MOTOR_ID = 0x01,          // CAN2 俯仰电机ID

} can_msg_id_e;

/**
 * @brief DJI电机测量数据结构体
 */
typedef struct
{   
    uint16_t pos;            // 编码器位置
    int16_t rpm;             // 电机转速
    int16_t current;         // 电机电流
    uint8_t temp;            // 电机温度

    float real_w;            // 实际角速度
    float real_pos;          // 实际位置

    float speed;             // 速度（可扩展）
    float accel;             // 加速度（可扩展）

    int16_t target_current;  // 目标电流
}dji_motor_measure_t;

/**
 * @brief 解析DJI电机CAN数据
 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
/**
 * @brief 解析底盘电机CAN数据
 */
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);

/* 外部变量声明 */
extern dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;//2摩擦轮，1拨蛋
extern dji_motor_measure_t chassis_motor[4];//四个轮子

/**
 * @brief 获取左摩擦轮电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_left();
/**
 * @brief 获取右摩擦轮电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_right();
/**
 * @brief 获取拨蛋电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_pull();
/**
 * @brief 获取底盘电机数据指针
 * @param i 电机编号（0~3）
 */
dji_motor_measure_t* get_chassis_motor(uint8_t i);


/////////////////////////////////////////////////////////////////////////////////
//dm电机统一数据结构体
/**
 * @brief 大妙电机测量数据结构体
 */
typedef struct
{
    uint8_t err;           // 错误状态
    float pos;             // 位置（弧度或度）
    float speed;           // 速度
    float tor;             // 转矩
    int8_t tmos_tmper;     // MOS温度
    int8_t coil_tmper;     // 线圈温度

    float target_tor;      // 目标转矩
}dm_motor_measure_t;

/**
 * @brief 解析大妙电机CAN数据
 */
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message);
/**
 * @brief 获取云台电机数据指针
 * @param i 电机编号
 */
dm_motor_measure_t* get_gimbal_motor(uint8_t i);


#endif // !__MYMOTOR_H__