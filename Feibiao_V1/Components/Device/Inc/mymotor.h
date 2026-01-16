/**
  ******************************************************************************
  * @file           : mymotor.h
  * @brief          : 自定义电机接口头文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 包含DJI电机、达妙电机、廉科电机的统一接口定义
  *                  支持发射机构、云台、底盘等多种电机控制需求
  ******************************************************************************
  */

#ifndef __MYMOTOR_H__
#define __MYMOTOR_H__

#include "main.h"

/* CAN通信ID定义 */
/**
 * @brief CAN消息ID枚举
 * @note GM6020电流给定值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A
 *       转矩常数 741 mN·m/A
 */
typedef enum
{
    /* CAN1总线电机ID定义 */
    CAN2_SHOOT_MOTOR_LEFT_ID = 0x202,   // 左摩擦轮电机ID
    CAN2_SHOOT_MOTOR_RIGHT_ID = 0x201,  // 右摩擦轮电机ID
    CAN2_SHOOT_PULL_MOTOR_ID = 0x203,   // 拨弹电机ID
    
    CAN1_CMD_ALL_ID = 0x200,            // CAN1发送命令帧ID

    /* CAN2总线电机ID定义(预留) */

    CAN2_CMD_ALL_ID =0x200,                 // CAN2发送命令帧ID(预留)


} can_msg_id_e;

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

/* DJI电机数据解析函数声明 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);

/* DJI电机实例声明 */
extern dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;  // 发射机构：2摩擦轮，1拨弹

/* DJI电机访问函数声明 */
dji_motor_measure_t* get_shoot_motor_left();   // 获取左摩擦轮电机指针
dji_motor_measure_t* get_shoot_motor_right();  // 获取右摩擦轮电机指针
dji_motor_measure_t* get_shoot_motor_pull();   // 获取拨弹电机指针

/////////////////////////////////////////////////////////////////////////////////


#endif // !__MYMOTOR_H__