#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "main.h"

// 机器人整体状态
typedef enum {
    ROBOT_STATE_INIT,        // 初始化状态
    ROBOT_STATE_NORMAL,      // 正常运行
    ROBOT_STATE_ERROR,       // 错误状态
    ROBOT_STATE_PROTECT     // 保护状态
} robot_state_e;

// 底盘状态
typedef enum {
    CHASSIS_STATE_DISABLE,           // 底盘失能
    CHASSIS_STATE_FOLLOW,           // 跟随云台
    CHASSIS_STATE_ROTATION,         // 小陀螺模式
    CHASSIS_STATE_FREE              // 自由移动模式
} chassis_state_e;

// 云台状态
typedef enum {
    GIMBAL_STATE_DISABLE,           // 云台失能
    GIMBAL_STATE_ABSOLUTE_ANGLE,    // 绝对角度控制
    GIMBAL_STATE_RELATIVE_ANGLE,    // 相对角度控制
    GIMBAL_STATE_FREE               // 自由控制
} gimbal_state_e;

// 发射机构状态
typedef enum {
    SHOOT_STATE_DISABLE,            // 发射机构失能
    SHOOT_STATE_READY,             // 就绪状态
    SHOOT_STATE_FIRING,            // 发射状态
    SHOOT_STATE_COOLING            // 冷却状态
} shoot_state_e;

// 机器人状态管理结构体
typedef struct {
    robot_state_e   robot_state;
    chassis_state_e chassis_state;
    gimbal_state_e  gimbal_state;
    shoot_state_e   shoot_state;

    struct {
        float vx;              // 底盘前后速度
        float vy;              // 底盘左右速度
        float wz;              // 底盘旋转速度
        float gimbal_pitch;    // 云台pitch角度
        float gimbal_yaw;      // 云台yaw角度
        bool  is_shooting;     // 是否在发射
    } control_input;          // 控制输入数据

    struct {
        uint8_t chassis_online : 1;
        uint8_t gimbal_online : 1;
        uint8_t shooter_online : 1;
        uint8_t imu_online : 1;
    } device_state;           // 设备在线状态
} robot_state_t;

// 状态机转换函数声明
void robot_state_init(robot_state_t* robot);
void robot_state_update(robot_state_t* robot);
bool robot_state_check(robot_state_t* robot);

extern robot_state_t g_robot_state;

#endif // ROBOT_STATE_H
