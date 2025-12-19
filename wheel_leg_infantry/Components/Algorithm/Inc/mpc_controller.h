/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : mpc_controller.h
 * @brief          : Model Predictive Control (MPC) Module for Wheeled-Legged Robot
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

/* ==================== 配置宏定义 ==================== */
// MPC参数
#define MPC_HORIZON_N   8     // 预测时域（步数）
#define MPC_STATE_DIM   3     // 状态维度 [x, x_dot, phi]
#define MPC_CONTROL_DIM 2     // 控制维度 [T_wheel, l_leg_rate]
#define MPC_SAMPLE_TIME 0.02f // 采样周期20ms（50Hz）

// 权重矩阵（对角元素）
#define MPC_Q_X       10.0f // 位置权重
#define MPC_Q_XDOT    5.0f  // 速度权重
#define MPC_Q_PHI     20.0f // 姿态权重
#define MPC_R_TORQUE  1.0f  // 力矩代价
#define MPC_R_LEGRATE 0.5f  // 腿长变化率代价

// 约束
#define MPC_TORQUE_MIN   -10.0f // Nm
#define MPC_TORQUE_MAX   10.0f  // Nm
#define MPC_LEG_RATE_MIN -0.5f  // m/s（收缩）
#define MPC_LEG_RATE_MAX 0.5f   // m/s（伸展）

// QP求解器参数
#define MPC_MAX_ITERATIONS 15    // 最大迭代次数（平衡精度与速度）
#define MPC_TOLERANCE      1e-3f // 收敛容差
#define MPC_GRADIENT_STEP  0.01f // 梯度下降步长

// 模型参数（简化线性化）
#define MPC_MODEL_MASS       4.3f  // 机器人总质量（kg）
#define MPC_MODEL_INERTIA    0.15f // 机体转动惯量（kg·m²）
#define MPC_MODEL_LEG_LENGTH 0.20f // 标称腿长（m）

/* ==================== 数据结构 ==================== */
/**
 * @brief MPC控制器状态结构体
 */
typedef struct {
    // ========== 输入：当前状态 ==========
    float state_current[MPC_STATE_DIM];   // [x, x_dot, phi]
    float state_reference[MPC_STATE_DIM]; // 期望状态
    float leg_length_current;             // 当前腿长（用于模型线性化）

    // ========== 输出：优化结果 ==========
    float control_output[MPC_CONTROL_DIM]; // 第一步控制量 [T_wheel, l_rate]
    float velocity_reference;              // 速度参考（供LQR跟踪）
    float leg_length_reference;            // 腿长参考

    // ========== 内部：线性化模型 ==========
    // 离散状态空间：x[k+1] = A*x[k] + B*u[k]
    float A[MPC_STATE_DIM * MPC_STATE_DIM];   // 状态转移矩阵
    float B[MPC_STATE_DIM * MPC_CONTROL_DIM]; // 控制矩阵

    // ========== QP问题数据 ==========
    // 标准形式：min 0.5*u'*H*u + g'*u  s.t. lb <= u <= ub
    float H[MPC_CONTROL_DIM * MPC_HORIZON_N * MPC_CONTROL_DIM * MPC_HORIZON_N]; // Hessian矩阵
    float g[MPC_CONTROL_DIM * MPC_HORIZON_N];                                   // 梯度向量
    float solution[MPC_CONTROL_DIM * MPC_HORIZON_N];                            // 优化变量

    // ========== 状态信息 ==========
    bool initialized;     // 是否已初始化
    uint32_t solve_count; // 求解次数（统计）
    float solve_time_ms;  // 求解耗时（ms，调试用）
    bool solve_success;   // 上次求解是否成功

} MPC_Controller_t;

/* ==================== 函数接口 ==================== */

/**
 * @brief 初始化MPC控制器
 * @param mpc 控制器结构体指针
 * @note 初始化模型参数、权重矩阵等
 */
void MPC_Controller_Init(MPC_Controller_t *mpc);

/**
 * @brief 更新线性化模型（在工作点处）
 * @param mpc 控制器结构体
 * @param leg_length 当前腿长（m）
 * @param velocity 当前速度（m/s）
 * @note 根据当前状态重新线性化A、B矩阵
 */
void MPC_UpdateModel(MPC_Controller_t *mpc, float leg_length, float velocity);

/**
 * @brief 求解MPC问题
 * @param mpc 控制器结构体
 * @return 是否成功求解
 * @note 耗时操作，建议在低频任务中调用（50Hz）
 * @note 调用前需先设置state_current、state_reference
 */
bool MPC_Solve(MPC_Controller_t *mpc);

/**
 * @brief 获取优化后的控制量（第一步）
 * @param mpc 控制器结构体
 * @param wheel_torque 输出：轮子力矩参考（Nm）
 * @param leg_length_rate 输出：腿长变化率参考（m/s）
 */
void MPC_GetControl(const MPC_Controller_t *mpc,
                    float *wheel_torque,
                    float *leg_length_rate);

/**
 * @brief 获取速度参考值（供LQR跟踪）
 * @param mpc 控制器结构体
 * @return 速度参考值（m/s）
 */
float MPC_GetVelocityReference(const MPC_Controller_t *mpc);

/**
 * @brief 获取腿长参考值
 * @param mpc 控制器结构体
 * @return 腿长参考值（m）
 */
float MPC_GetLegLengthReference(const MPC_Controller_t *mpc);

/**
 * @brief 打印调试信息
 * @param mpc 控制器结构体
 */
void MPC_PrintDebugInfo(const MPC_Controller_t *mpc);

#ifdef __cplusplus
}
#endif

#endif // MPC_CONTROLLER_H
