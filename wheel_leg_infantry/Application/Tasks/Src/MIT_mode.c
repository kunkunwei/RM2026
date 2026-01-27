//
// Created by kun on 2026/1/27.
//

#include "MIT_mode.h"
#include <stdint.h>
#include "user_lib.h"
JointTorqueCtrl joint_ctrl[4] =
{
    {.kd = 0.1f, .tau_limit = 2.0f},   // LF
    {.kd = 0.1f, .tau_limit = 2.0f},   // LB
    {.kd = 0.1f, .tau_limit = 2.0f},   // RF
    {.kd = 0.1f, .tau_limit = 2.0f},   // RB
};
/**
 * @brief 将期望力矩转换为电机电流
 * @param torque_desired 期望输出力矩 (N·m)，在减速器输出端
 * @param spec 电机规格参数
 * @return 电机相电流 (A)，考虑减速比
 */
float torque_to_motor_current(float torque_desired) {
    // 考虑减速比：电机端力矩 = 输出端力矩 / 减速比
    float motor_torque = torque_desired /gear_ratio;

    // 根据力矩常数计算电流：I = τ / Kt
    float motor_current = motor_torque /torque_constant;

    return motor_current;
}

/**
 * @brief 将电机电流转换为CAN指令值
 * @param current 电机相电流 (A)
 * @param spec 电机规格参数
 * @return CAN指令值 (-16384 ~ 16384)
 */
int16_t current_to_can_value(float current) {
    // 限制电流在允许范围内
    current = float_constrain(current, -current_max, current_max);

    // 转换为CAN值
    float can_float = current * current_to_can_scale;

    // 四舍五入为整数
    return (int16_t)(can_float + (can_float >= 0 ? 0.5f : -0.5f));
}
static inline float clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/**
 * @brief 关节扭矩 → CAN 电流指令
 * @param ctrl      关节控制参数
 * @param tau_des   上层给定的期望关节扭矩 (Nm)
 * @param joint_vel 实际关节角速度 (rad/s)
 * @return int16_t  CAN 电流指令 (-16384 ~ 16384)
 */
int16_t Joint_Torque_To_CAN(
        const JointTorqueCtrl *ctrl,
        float tau_des,
        float joint_vel,
        float *current_set)
{
    /* ---------- 1. 关节阻尼 ---------- */
    float tau_cmd = tau_des - ctrl->kd * joint_vel;

    /* ---------- 2. 扭矩限幅 ---------- */
    tau_cmd = clampf(tau_cmd,
                     -ctrl->tau_limit,
                      ctrl->tau_limit);

    /* ---------- 3. 扭矩 → 电流 ---------- */
    // τ = gear * Kt * I
    float motor_current =
        tau_cmd / (DM8009_GEAR * DM8009_KT);

    /* ---------- 4. 电流限幅 ---------- */
    motor_current = clampf(motor_current,
                           -DM8009_I_MAX,
                            DM8009_I_MAX);
    *current_set=motor_current;
    /* ---------- 5. 电流 → CAN ---------- */
    float can_f = motor_current * CAN_I_SCALE;

    return (int16_t)(can_f);
}
