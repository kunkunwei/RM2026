//
// Created by kun on 2026/1/27.
//

#ifndef __MIT_MODE_H
#define __MIT_MODE_H

// DM8009电机电流环参数

#define torque_constant  1.0f      // 力矩常数 Kt (N·m/A)
#define gear_ratio  9.0f            // 9:1减速比
#define current_max  41.044777f     // 最大电流 (A)
#define current_to_can_scale  399.1738096177255391f  // 电流到CAN的缩放系数:16384.0f / 41.044777f

// ===== 电机 / 关节物理参数 =====
#define DM8009_KT        0.14f      // Nm/A（电机端，保守值）
#define DM8009_GEAR      9.0f       // 减速比
#define DM8009_I_MAX     41.044777f      // 峰值电流（A）

// ===== 一拖四协议 =====
#define CAN_I_MAX        16384.0f
#define CAN_I_SCALE     (CAN_I_MAX / DM8009_I_MAX) // ≈ 399

// MIT力矩环参数
#define Kp_pos  0.0f                    // 位置刚度
#define Kd_pos  1.2f                     // 位置阻尼
#define Ki_pos  0.0f                     // 位置积分

#define Kp_vel  0.008994053f             // 速度刚度
#define Ki_vel  0.01963495f              // 速度积分
#include <stdint.h>

typedef struct
{

    float kd;          // 关节阻尼 (Nm / (rad/s))
    float tau_limit;   // 关节最大输出扭矩 (Nm)
    float vel_integral;
} JointTorqueCtrl;
extern JointTorqueCtrl joint_ctrl[4];
int16_t Joint_Torque_To_CAN(
        const JointTorqueCtrl *ctrl,
        float tau_des,
        float joint_vel,
        float *current_set);
float torque_to_motor_current(float torque_desired);
int16_t current_to_can_value(float current);

#endif //__MIT_MODE_H