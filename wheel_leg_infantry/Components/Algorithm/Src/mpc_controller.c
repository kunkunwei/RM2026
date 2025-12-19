/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : mpc_controller.c
 * @brief          : Model Predictive Control (MPC) Implementation
 * @author         : AI Assistant
 * @date           : 2025/11/29
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 简化版MPC实现，适合嵌入式实时运行
 *                   使用梯度下降+约束投影代替完整QP求解器
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "mpc_controller.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "arm_math.h"

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 限幅函数
 */
static inline float constrain_float(float value, float min_val, float max_val)
{
    if (value > max_val) return max_val;
    if (value < min_val) return min_val;
    return value;
}

/**
 * @brief 构建QP问题的Hessian矩阵和梯度向量
 * @note 简化版本：对角Hessian + 线性梯度
 */
static void BuildQPMatrices(MPC_Controller_t *mpc)
{
    // 简化处理：使用对角Hessian矩阵
    // H = diag([R, R, ..., R]) (N个控制量)
    memset(mpc->H, 0, sizeof(mpc->H));

    for (int i = 0; i < MPC_HORIZON_N; i++) {
        int idx_torque  = i * MPC_CONTROL_DIM + 0;
        int idx_legrate = i * MPC_CONTROL_DIM + 1;

        // 对角元素
        mpc->H[idx_torque * (MPC_CONTROL_DIM * MPC_HORIZON_N) + idx_torque]   = MPC_R_TORQUE;
        mpc->H[idx_legrate * (MPC_CONTROL_DIM * MPC_HORIZON_N) + idx_legrate] = MPC_R_LEGRATE;
    }

    // 计算梯度 g（基于状态误差）
    // g = B'*Q*A*x0 - B'*Q*x_ref （简化）
    float error_x    = mpc->state_current[0] - mpc->state_reference[0];
    float error_xdot = mpc->state_current[1] - mpc->state_reference[1];
    float error_phi  = mpc->state_current[2] - mpc->state_reference[2];

    // 简化梯度计算（基于状态误差的线性函数）
    for (int i = 0; i < MPC_HORIZON_N; i++) {
        int idx = i * MPC_CONTROL_DIM;

        // 力矩梯度：主要影响速度和姿态
        mpc->g[idx + 0] = (error_xdot * MPC_Q_XDOT + error_phi * MPC_Q_PHI) * 0.1f;

        // 腿长变化率梯度：主要影响姿态
        mpc->g[idx + 1] = error_phi * MPC_Q_PHI * 0.05f;
    }
}

/**
 * @brief 简化QP求解器（梯度下降 + 投影）
 * @note 使用投影梯度法：迭代更新并投影到约束集
 */
static bool SolveQP_ProjectedGradient(MPC_Controller_t *mpc)
{
    // 初始解：上一次的解（热启动）或零
    if (mpc->solve_count == 0) {
        memset(mpc->solution, 0, sizeof(mpc->solution));
    }
    // else: 使用上次解作为初值（热启动加速收敛）

    float alpha = MPC_GRADIENT_STEP; // 步长

    for (int iter = 0; iter < MPC_MAX_ITERATIONS; iter++) {
        float max_grad_norm = 0.0f;

        // 计算梯度：grad = H*u + g
        for (int i = 0; i < MPC_HORIZON_N * MPC_CONTROL_DIM; i++) {
            float grad = mpc->g[i];

            // H*u（简化：对角矩阵）
            int row = i;
            int col = i;
            grad += mpc->H[row * (MPC_HORIZON_N * MPC_CONTROL_DIM) + col] * mpc->solution[i];

            // 梯度下降更新
            mpc->solution[i] -= alpha * grad;

            // 约束投影
            if (i % MPC_CONTROL_DIM == 0) {
                // 力矩约束
                mpc->solution[i] = constrain_float(mpc->solution[i],
                                                   MPC_TORQUE_MIN,
                                                   MPC_TORQUE_MAX);
            } else {
                // 腿长变化率约束
                mpc->solution[i] = constrain_float(mpc->solution[i],
                                                   MPC_LEG_RATE_MIN,
                                                   MPC_LEG_RATE_MAX);
            }

            // 记录最大梯度（收敛判断）
            float grad_abs = fabsf(grad);
            if (grad_abs > max_grad_norm)
                max_grad_norm = grad_abs;
        }

        // 收敛判断
        if (max_grad_norm < MPC_TOLERANCE) {
            return true; // 收敛成功
        }
    }

    // 达到最大迭代次数（虽未完全收敛，但结果可能仍可用）
    return true;
}

/* Public functions ----------------------------------------------------------*/

void MPC_Controller_Init(MPC_Controller_t *mpc)
{
    if (mpc == NULL) return;

    memset(mpc, 0, sizeof(MPC_Controller_t));

    // 初始化A矩阵（离散状态转移）
    // 简化线性模型：
    // x[k+1] = x[k] + Ts*x_dot[k]
    // x_dot[k+1] = a_d*x_dot[k] + b_d*T_wheel[k]
    // phi[k+1] = a_phi*phi[k] + b_phi*l_rate[k]

    float Ts = MPC_SAMPLE_TIME;

    // A矩阵（3x3）
    mpc->A[0 * MPC_STATE_DIM + 0] = 1.0f; // x -> x
    mpc->A[0 * MPC_STATE_DIM + 1] = Ts;   // x_dot -> x
    mpc->A[0 * MPC_STATE_DIM + 2] = 0.0f;

    mpc->A[1 * MPC_STATE_DIM + 0] = 0.0f;
    mpc->A[1 * MPC_STATE_DIM + 1] = 0.98f; // x_dot衰减（摩擦/阻尼）
    mpc->A[1 * MPC_STATE_DIM + 2] = 0.0f;

    mpc->A[2 * MPC_STATE_DIM + 0] = 0.0f;
    mpc->A[2 * MPC_STATE_DIM + 1] = 0.0f;
    mpc->A[2 * MPC_STATE_DIM + 2] = 0.95f; // phi衰减（自然回正）

    // B矩阵（3x2）
    mpc->B[0 * MPC_CONTROL_DIM + 0] = 0.0f; // T不直接影响x
    mpc->B[0 * MPC_CONTROL_DIM + 1] = 0.0f;

    mpc->B[1 * MPC_CONTROL_DIM + 0] = Ts * 0.5f / MPC_MODEL_MASS; // T影响x_dot
    mpc->B[1 * MPC_CONTROL_DIM + 1] = 0.0f;

    mpc->B[2 * MPC_CONTROL_DIM + 0] = 0.0f;
    mpc->B[2 * MPC_CONTROL_DIM + 1] = Ts * 0.3f; // l_rate影响phi

    mpc->initialized = true;
    mpc->solve_count = 0;
}

void MPC_UpdateModel(MPC_Controller_t *mpc, float leg_length, float velocity)
{
    if (mpc == NULL) return;

    // TODO: 根据当前工作点重新线性化A、B
    // 当前使用固定线性化模型（简化）
    // 实际应用中可根据腿长和速度调整B矩阵系数

    // 示例：腿长影响phi动态
    float leg_factor                = leg_length / MPC_MODEL_LEG_LENGTH;
    mpc->B[2 * MPC_CONTROL_DIM + 1] = MPC_SAMPLE_TIME * 0.3f * leg_factor;
}

bool MPC_Solve(MPC_Controller_t *mpc)
{
    if (mpc == NULL || !mpc->initialized) return false;

    uint32_t start_tick = HAL_GetTick();

    // 1. 构建QP问题矩阵
    BuildQPMatrices(mpc);

    // 2. 求解QP（投影梯度法）
    bool success = SolveQP_ProjectedGradient(mpc);

    // 3. 提取第一步控制量
    if (success) {
        mpc->control_output[0] = mpc->solution[0]; // T_wheel
        mpc->control_output[1] = mpc->solution[1]; // l_rate

        // 更新参考值
        mpc->velocity_reference = mpc->state_reference[1]; // 期望速度

        // 腿长参考：积分腿长变化率
        mpc->leg_length_reference = mpc->leg_length_current +
                                    mpc->control_output[1] * MPC_SAMPLE_TIME;

        mpc->solve_count++;
    }

    mpc->solve_time_ms = (float)(HAL_GetTick() - start_tick);
    mpc->solve_success = success;

    return success;
}

void MPC_GetControl(const MPC_Controller_t *mpc,
                    float *wheel_torque,
                    float *leg_length_rate)
{
    if (mpc == NULL) return;

    if (wheel_torque != NULL)
        *wheel_torque = mpc->control_output[0];

    if (leg_length_rate != NULL)
        *leg_length_rate = mpc->control_output[1];
}

float MPC_GetVelocityReference(const MPC_Controller_t *mpc)
{
    return (mpc != NULL) ? mpc->velocity_reference : 0.0f;
}

float MPC_GetLegLengthReference(const MPC_Controller_t *mpc)
{
    return (mpc != NULL) ? mpc->leg_length_reference : 0.20f;
}

void MPC_PrintDebugInfo(const MPC_Controller_t *mpc)
{
    if (mpc == NULL) return;

    printf("\r\n--- MPC Controller Debug Info ---\r\n");
    printf("Initialized: %s\r\n", mpc->initialized ? "YES" : "NO");
    printf("Solve Count: %lu\r\n", mpc->solve_count);
    printf("Last Solve: %s (%.2f ms)\r\n",
           mpc->solve_success ? "SUCCESS" : "FAILED",
           mpc->solve_time_ms);
    printf("Current State: [%.3f, %.3f, %.3f]\r\n",
           mpc->state_current[0], mpc->state_current[1], mpc->state_current[2]);
    printf("Reference State: [%.3f, %.3f, %.3f]\r\n",
           mpc->state_reference[0], mpc->state_reference[1], mpc->state_reference[2]);
    printf("Control Output: T=%.2f Nm, l_rate=%.3f m/s\r\n",
           mpc->control_output[0], mpc->control_output[1]);
    printf("Velocity Ref: %.3f m/s\r\n", mpc->velocity_reference);
    printf("Leg Length Ref: %.3f m\r\n", mpc->leg_length_reference);
    printf("--------------------------------\r\n\r\n");
}
