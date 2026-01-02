// mpc_controller.c
#include "mpc_controller.h"
// #include "dwt.h"          // 时间测量
#include <math.h>
#include <string.h>

// ==================== 内部函数声明 ====================
static void build_qp_problem(MPCController_t *mpc);
static bool solve_qp_fast_gradient(MPCController_t *mpc);
static void predict_trajectory(MPCController_t *mpc);
static void apply_constraints(MPCController_t *mpc);
static void update_system_matrices(float slip_rate, float A[], float B[]);
static float box_projection(float x, float lower, float upper);

// ==================== 默认参数 ====================
static const float DEFAULT_Q[MPC_STATE_DIM] = {
    10.0f,   // θ权重
    2.0f,    // θ_dot权重
    0.1f,    // x权重
    1.0f,    // x_dot权重
    8.0f,    // φ权重
    1.5f,    // φ_dot权重
    5.0f     // s权重（滑移率）
};

static const float DEFAULT_R[MPC_CONTROL_DIM] = {
    0.01f,   // τ_wheel权重
    0.02f    // τ_joint权重
};

// ==================== 公共函数实现 ====================

void MPC_Init(MPCController_t *mpc, float dt)
{
    if (!mpc) return;

    memset(mpc, 0, sizeof(MPCController_t));

    // 基础配置
    mpc->dt = dt;
    mpc->torque_joint_max = 15.0f;
    mpc->torque_wheel_max = 8.0f;
    mpc->slip_rate_max = 0.3f;

    // 默认权重
    memcpy(mpc->Q, DEFAULT_Q, sizeof(DEFAULT_Q));
    memcpy(mpc->R, DEFAULT_R, sizeof(DEFAULT_R));
    memcpy(mpc->Qf, DEFAULT_Q, sizeof(DEFAULT_Q));

    // 放大终端权重
    for (int i = 0; i < MPC_STATE_DIM; i++) {
        mpc->Qf[i] *= 2.0f;
    }

    // 初始化边界约束
    int total_vars = MPC_CTRL_HORIZON * MPC_CONTROL_DIM;
    for (int i = 0; i < total_vars; i++) {
        if (i % 2 == 0) {
            // 轮子力矩约束
            mpc->lb[i] = -mpc->torque_wheel_max;
            mpc->ub[i] = mpc->torque_wheel_max;
        } else {
            // 关节力矩约束
            mpc->lb[i] = -mpc->torque_joint_max;
            mpc->ub[i] = mpc->torque_joint_max;
        }
    }

    MPC_Reset(mpc);
}

void MPC_SetState(MPCController_t *mpc, const float *state)
{
    if (!mpc || !state) return;
    memcpy(mpc->x, state, MPC_STATE_DIM * sizeof(float));
}

void MPC_SetReference(MPCController_t *mpc, const float *ref)
{
    if (!mpc || !ref) return;
    memcpy(mpc->x_ref, ref, MPC_STATE_DIM * sizeof(float));
}

bool MPC_Solve(MPCController_t *mpc)
{
    if (!mpc) return false;

    // DWT_StartTiming();

    // 1. 构建QP问题
    build_qp_problem(mpc);

    // 2. 求解QP
    bool solved = solve_qp_fast_gradient(mpc);

    // 3. 保存求解时间
    // mpc->solve_time_us = DWT_StopTiming();

    // 4. 更新统计
    mpc->solve_count++;
    mpc->feasible = solved;
    if (!solved) {
        mpc->fail_count++;
        // 使用上一次的解作为降级
        return false;
    }

    // 5. 提取第一个控制量
    mpc->u_opt[0] = mpc->z[0];  // τ_wheel
    mpc->u_opt[1] = mpc->z[1];  // τ_joint

    return true;
}

void MPC_GetControl(const MPCController_t *mpc, float *u_wheel, float *u_joint)
{
    if (!mpc || !u_wheel || !u_joint) return;

    *u_wheel = mpc->u_opt[0];
    *u_joint = mpc->u_opt[1];
}

void MPC_Reset(MPCController_t *mpc)
{
    if (!mpc) return;

    // 清零控制序列
    memset(mpc->z, 0, MPC_CTRL_HORIZON * MPC_CONTROL_DIM * sizeof(float));
    memset(mpc->U_opt, 0, sizeof(mpc->U_opt));

    // 重置状态
    mpc->feasible = true;
    mpc->iterations = 0;
}

void MPC_UpdateSlipModel(MPCController_t *mpc, float slip_rate)
{
    // 这里可以更新系统矩阵，但为了简化，我们先用固定模型
    // 如果需要，可以实现：
    // update_system_matrices(slip_rate, A, B);
    (void)mpc;
    (void)slip_rate;
}

// ==================== QP求解核心 ====================

static bool solve_qp_fast_gradient(MPCController_t *mpc)
{
    int n = MPC_CTRL_HORIZON * MPC_CONTROL_DIM;

    // 对角预条件（简化）
    float alpha = 0.1f;  // 步长

    for (mpc->iterations = 0; mpc->iterations < MPC_MAX_ITER; mpc->iterations++) {
        // 计算梯度: g = H*z + g
        // arm_mat_mult_f32(&H_mat, &z_vec, &grad_vec);
        // arm_add_f32(grad_vec.pData, mpc->g, mpc->grad, n);

        // 梯度更新: z_new = z - alpha * grad
        arm_scale_f32(mpc->grad, -alpha, mpc->z_new, n);
        arm_add_f32(mpc->z, mpc->z_new, mpc->z_new, n);

        // 投影到约束集
        for (int i = 0; i < n; i++) {
            mpc->z_new[i] = box_projection(mpc->z_new[i], mpc->lb[i], mpc->ub[i]);
        }

        // 检查收敛
        float diff_norm = 0.0f;
        for (int i = 0; i < n; i++) {
            float diff = mpc->z_new[i] - mpc->z[i];
            diff_norm += diff * diff;
        }

        if (sqrtf(diff_norm) < MPC_TOLERANCE) {
            memcpy(mpc->z, mpc->z_new, n * sizeof(float));
            return true;
        }

        // 更新
        memcpy(mpc->z, mpc->z_new, n * sizeof(float));
    }

    return false;  // 超过最大迭代次数
}

// ==================== QP问题构建 ====================

static void build_qp_problem(MPCController_t *mpc)
{
    // 简化：这里构建一个二次代价函数
    // 实际应根据预测模型构建

    int n = MPC_CTRL_HORIZON * MPC_CONTROL_DIM;

    // 初始化H为对角矩阵
    memset(mpc->H, 0, n * n * sizeof(float));

    for (int i = 0; i < n; i++) {
        int ctrl_idx = i % MPC_CONTROL_DIM;
        mpc->H[i * n + i] = mpc->R[ctrl_idx];

        // 添加一点正则化
        mpc->H[i * n + i] += 0.001f;
    }

    // 简化：梯度项设为0（实际应根据状态偏差计算）
    memset(mpc->g, 0, n * sizeof(float));
}

// ==================== 辅助函数 ====================

static float box_projection(float x, float lower, float upper)
{
    if (x < lower) return lower;
    if (x > upper) return upper;
    return x;
}

static void predict_trajectory(MPCController_t *mpc)
{
    // 简化的预测（实际应根据动力学模型）
    memcpy(mpc->X_pred[0], mpc->x, MPC_STATE_DIM * sizeof(float));

    for (int k = 0; k < MPC_PRED_HORIZON; k++) {
        // 简化：假设状态不变（实际应集成动力学）
        memcpy(mpc->X_pred[k+1], mpc->X_pred[k], MPC_STATE_DIM * sizeof(float));
    }
}