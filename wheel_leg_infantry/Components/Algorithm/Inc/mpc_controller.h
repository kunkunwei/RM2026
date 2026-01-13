/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : mpc_controller.h
 * @brief          : 轮足机器人模型预测控制 (MPC) 模块
 * @author         : AI Assistant
 * @date           : 2025/11/29
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 分层MPC控制器 - 输出参考轨迹供LQR跟踪
 *                   简化线性MPC + 小型QP求解器（主动集法）
 *                   适用于STM32F407@168MHz实时运行
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "config.h"

/* ==================== 配置参数 ==================== */
#define MPC_STATE_DIM       7      // 状态维度
#define MPC_CONTROL_DIM     2      // 控制维度
#define MPC_PRED_HORIZON    12     // 预测时域 N=12
#define MPC_CTRL_HORIZON    6      // 控制时域 M=6
#define MPC_MAX_ITER        20     // 最大迭代次数
#define MPC_TOLERANCE       1e-4f  // 收敛容忍度

/* ==================== 数据结构 ==================== */

/**
 * @brief MPC控制器（高度集成，单文件实现）
 */
typedef struct {
    // --- 配置参数 ---
    float dt;                      // 采样时间 [s]
    float torque_joint_max;        // 关节力矩上限 [N·m]
    float torque_wheel_max;        // 轮子力矩上限 [N·m]
    float slip_rate_max;           // 最大允许滑移率

    // --- 权重矩阵 ---
    float Q[MPC_STATE_DIM];        // 状态权重（对角，简化存储）
    float R[MPC_CONTROL_DIM];      // 控制权重（对角）
    float Qf[MPC_STATE_DIM];       // 终端权重

    // --- 当前状态 ---
    float x[MPC_STATE_DIM];        // 当前状态 [θ, θ_dot, x, x_dot, φ, φ_dot, s]
    float x_ref[MPC_STATE_DIM];    // 参考状态
    float u_opt[MPC_CONTROL_DIM];  // 最优控制输出 [τ_wheel, τ_joint]

    // --- 工作内存（静态分配，避免动态内存）---
    // 预测状态序列
    float X_pred[MPC_PRED_HORIZON+1][MPC_STATE_DIM];

    // 控制序列
    float U_opt[MPC_CTRL_HORIZON][MPC_CONTROL_DIM];

    // QP求解工作内存
    float H[(MPC_CTRL_HORIZON*MPC_CONTROL_DIM) * (MPC_CTRL_HORIZON*MPC_CONTROL_DIM)];
    float g[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];
    float z[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];
    float z_new[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];
    float grad[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];

    // 边界约束
    float lb[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];
    float ub[MPC_CTRL_HORIZON*MPC_CONTROL_DIM];

    // --- 性能统计 ---
    uint32_t solve_time_us;        // 求解时间 [μs]
    uint16_t iterations;           // 实际迭代次数
    uint32_t solve_count;          // 总求解次数
    uint32_t fail_count;           // 求解失败次数
    bool_t feasible;               // 当前解是否可行

} MPCController_t;

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化MPC控制器
 * @param mpc MPC控制器指针
 * @param dt 采样时间 [s]
 */
void MPC_Init(MPCController_t *mpc, float dt);

/**
 * @brief 设置当前状态
 * @param mpc MPC控制器指针
 * @param state 状态数组 [θ, θ_dot, x, x_dot, φ, φ_dot, s]
 */
void MPC_SetState(MPCController_t *mpc, const float *state);

/**
 * @brief 设置参考状态
 * @param mpc MPC控制器指针
 * @param ref 参考状态数组
 */
void MPC_SetReference(MPCController_t *mpc, const float *ref);

/**
 * @brief 执行MPC求解
 * @param mpc MPC控制器指针
 * @return true 求解成功，false 求解失败
 */
bool MPC_Solve(MPCController_t *mpc);

/**
 * @brief 获取最优控制量（调用MPC_Solve后使用）
 * @param mpc MPC控制器指针
 * @param u_wheel 输出：轮子力矩 [N·m]
 * @param u_joint 输出：关节力矩 [N·m]
 */
void MPC_GetControl(const MPCController_t *mpc, float *u_wheel, float *u_joint);

/**
 * @brief 重置MPC求解器
 * @param mpc MPC控制器指针
 */
void MPC_Reset(MPCController_t *mpc);

/**
 * @brief 设置滑移率相关参数（模型更新）
 * @param mpc MPC控制器指针
 * @param slip_rate 当前滑移率
 */
void MPC_UpdateSlipModel(MPCController_t *mpc, float slip_rate);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
}
#endif

#endif // MPC_CONTROLLER_H
