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
#define P_MIN -12.56f
#define P_MAX 12.56f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
/* CAN通信ID定义 */
/**
 * @brief CAN消息ID枚举
 * @note GM6020电流给定值范围：-16384~0~16384, 对应最大转矩电流范围 -3A~0~3A
 *       转矩常数 741 mN·m/A
 */
typedef enum
{
    /* CAN1总线电机ID定义 */

    CAN1_SHOOT_PULL_MOTOR_ID = 0x207,   // 拨弹电机ID
    CAN1_CHASSIS_MOTOR_1_ID = 0x201,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_2_ID = 0x202,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_3_ID = 0x203,   // 底盘电机ID
    CAN1_CHASSIS_MOTOR_4_ID = 0x204,   // 底盘电机ID

    CAN1_YAW_MOTOR_FB_ID=0x012,             // Yaw轴电机反馈ID

    /* CAN2总线电机ID定义 */
    CAN2_SHOOT_MOTOR_RIGHT_1_ID = 0x201,  // 右摩擦轮电机ID
    CAN2_SHOOT_MOTOR_LEFT_1_ID = 0x202,   // 左摩擦轮电机ID
    CAN2_SHOOT_MOTOR_RIGHT_2_ID = 0x203,  // 右摩擦轮电机ID
    CAN2_SHOOT_MOTOR_LEFT_2_ID = 0x204,   // 左摩擦轮电机ID

    CAN2_PITCH_MOTOR_FB_ID = 0x011,      // Pitch轴电机反馈ID
    /*控制ID*/
    CAN1_CMD_PULL_ID=0x1FF,                 // CAN1发送命令帧ID,控制底盘拨弹轮电机
    CAN1_CMD_ALL_ID=0x200,                 // CAN1发送命令帧ID,控制底盘电机
    CAN1_YAW_MOTOR_ID = 0x02,             // Yaw轴电机ID

    CAN2_PITCH_MOTOR_ID = 0x01,        // Pitch轴电机ID
    CAN2_CMD_ALL_ID = 0x200,            // CAN2发送命令帧ID,控制摩擦轮机构电机



} can_msg_id_e;

/**
 * @brief DJI电机测量数据结构体
 * @details 用于存储DJI电机的反馈数据和计算得到的物理量
 */
typedef struct
{
    uint16_t last_ecd;       //上次转子机械角度 (0-8191)，用于计算位置变化量
    uint16_t ecd;          // 编码器位置原始值(0-8191)
    int16_t rpm;           // 转速原始值(rpm)
    int16_t current;       // 电流原始值(mA)
    uint8_t temp;          // 温度(°C)

    float real_w;          // 实际角速度(rad/s)
    float real_pos;        // 实际位置(rad)

    // float speed;           // 目标速度
    // float accel;           // 加速度
    
    int16_t target_current; // 目标电流值
}dji_motor_measure_t;

/* DJI电机数据解析函数声明 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message);

/* DJI电机实例已在 mymotor.c 中声明为 static，外部通过下方 getter 访问 */
/* DJI电机访问函数声明 */
dji_motor_measure_t* get_shoot_motor(uint8_t i);
dji_motor_measure_t* get_shoot_motor_pull();   // 获取拨弹电机指针
dji_motor_measure_t* get_chassis_motor(uint8_t i);  // 获取底盘电机指针，i=0~3分别对应4个底盘电机

/**
 * @brief 将电机 CAN 接收回调注册到 BSP 层
 * @note  应在 BSP_CAN_Init() 之后、任务启动之前调用
 */
void mymotor_register_can_callbacks(void);
/////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 达妙DM8009电机统一数据结构体
 * @details 用于存储达妙电机的反馈数据和控制参数
 */
typedef struct
{
    uint8_t id;             // 控制器id
    uint8_t err;            // 故障类型
    uint16_t pos_int;
    uint16_t vel_int;
    uint16_t tor_int;
    float pos;              // 位置信息(rad)
    float vel;              // 速度信息(rad/s)
    float tor;              // 扭矩信息(N·m)
    float T_Mos;            // MOS管上的温度信息(°C)
    float T_Rotor;          // 电机线圈的温度信息(°C)
    //

    float target_tor;      // 目标扭矩(N·m)

    TickType_t update_time; // 数据更新时间戳
}dm_motor_measure_t;

/* 达妙电机数据解析函数声明 */
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message);

/* 达妙电机访问函数声明 */

dm_motor_measure_t* get_pitch_motor();  // 获取Pitch轴电机指针
dm_motor_measure_t* get_yaw_motor();  // 获取Yaw轴电机指针

#endif // !__MYMOTOR_H__