// leg_angular_predictor.c
#include "leg_angular_predictor.h"

// #include <string.h>

/* ==================== 内部函数 ==================== */

// 计算单腿KF的系统矩阵（根据腿长）
static void calc_leg_kf_matrices(KalmanFilter_Info_TypeDef *kf, float leg_length)
{
    if (leg_length < 0.01f) return;

    float dt = PREDICTOR_DT;
    // float m = LEG_MASS_EFFECTIVE;
    float g = 9.81f;
    float r = WHEEL_RADIUS;
    // 使用拟合的 m_eff(l)
    float m = 1.1896f - 2.2927f*leg_length + 0.9166f*leg_length*leg_length;
    // 连续时间系统矩阵
    float a21 = g / leg_length;                     // 重力项
    float b_leg = 1.0f / (m * leg_length * leg_length);   // 关节力矩系数
    float b_wheel = 1.0f / (m * leg_length * r);          // 轮子力矩系数

    // 离散化（前向欧拉）
    float A[4] = {1.0f, dt,
              a21 * dt, 1.0f};
    float B[4] = {0.0f, 0.0f, b_leg * dt, b_wheel * dt};

    // 固定参数
    float Q[4] = {10.0f, 0.1f,
                0.1f, 50.0f};     // 过程噪声
    float R[4] = {0.02f, 0.0f,
                0.0f, 0.1f};      // 观测噪声
    float P[4] = {0.1f, 0.0f,
                0.0f, 0.1f};       // 初始协方差
    float H[4] = {1.0f, 0.0f,
                0.0f, 1.0f};       // 观测矩阵 H = I

    memcpy(kf->Data.A, A, sizeof(A));
    memcpy(kf->Data.B, B, sizeof(B));
    memcpy(kf->Data.Q, Q, sizeof(Q));
    memcpy(kf->Data.R, R, sizeof(R));
    memcpy(kf->Data.P, P, sizeof(P));
    memcpy(kf->Data.H, H, sizeof(H));
}

// 更新单腿KF的模型参数（如果腿长变化）
// static void update_leg_kf_params(KalmanFilter_Info_TypeDef *kf, float new_leg_length[2])
// {
//     static float last_leg_length[2] = {0.0f,0.0f};
//
//     if (fabsf(new_leg_length[0] - last_leg_length[0]) > 0.001f) {
//         last_leg_length[0] = new_leg_length[0];
//         calc_leg_kf_matrices(kf, new_leg_length[0]);
//     }
//     if (fabsf(new_leg_length[1] - last_leg_length[1]) > 0.001f) {
//         last_leg_length[1] = new_leg_length[1];
//         calc_leg_kf_matrices(kf, new_leg_length[1]);
//     }
// }

/* ==================== 公共函数实现 ==================== */

void LegPredictor_Init(LegPredictor_t *predictor,
                      const chassis_move_t *chassis,
                      float K_adjust)
{
    if (!predictor || !chassis) return;

    memset(predictor, 0, sizeof(LegPredictor_t));

    // 初始化左右腿KF
    Kalman_Filter_Init(&predictor->kf_left, 2, 0, 2);
    Kalman_Filter_Init(&predictor->kf_right, 2, 0, 2);

    // 设置系统矩阵（基于当前腿长）
    calc_leg_kf_matrices(&predictor->kf_left, chassis->left_leg.leg_length);
    calc_leg_kf_matrices(&predictor->kf_right, chassis->right_leg.leg_length);

    // 设置初始状态
    float init_left[2] = {
        chassis->left_leg.leg_angle - PI/2 - chassis->chassis_pitch,
        chassis->left_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET)
    };

    float init_right[2] = {
        chassis->right_leg.leg_angle - PI/2 - chassis->chassis_pitch,
        chassis->right_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET)
    };

    memcpy(predictor->kf_left.Output, init_left, sizeof(init_left));
    memcpy(predictor->kf_right.Output, init_right, sizeof(init_right));

    // 设置参数
    predictor->K_adjust = K_adjust;
}

void LegPredictor_Update(LegPredictor_t *predictor,
                        const chassis_move_t *chassis,
                        float leg_torque,
                        float wheel_torque,
                        float err_torque)
{
    if (!predictor || !chassis) return;

    // ========== 1. 更新模型参数（如果腿长变化） ==========
    // float new_leg_length[2]={
    //     chassis->left_leg.leg_length,
    //     chassis->right_leg.leg_length
    // };
    // update_leg_kf_params(&predictor->kf_left, chassis->left_leg.leg_length);
    // update_leg_kf_params(&predictor->kf_right, chassis->right_leg.leg_length);
    static float last_left_leg_length=0.0f;
    static float last_right_leg_length=0.0f;

    if (fabsf(chassis->left_leg.leg_length - last_left_leg_length) > 0.001f) {
        last_left_leg_length = chassis->left_leg.leg_length;
        calc_leg_kf_matrices(&predictor->kf_left, chassis->left_leg.leg_length);
    }
    if (fabsf(chassis->right_leg.leg_length - last_right_leg_length) > 0.001f) {
        last_right_leg_length = chassis->right_leg.leg_length;
        calc_leg_kf_matrices(&predictor->kf_right, chassis->right_leg.leg_length);
    }
    // ========== 2. 设置控制输入 ==========
    float u_left[2] = {
        leg_torque - err_torque,           // 左腿关节力矩
        // wheel_torque * 0.5f                // 左轮力矩（均分）如果在LQR更新之前调用就均分
    };

    float u_right[2] = {
        leg_torque + err_torque,           // 右腿关节力矩
        // wheel_torque * 0.5f                // 右轮力矩（均分）如果在LQR更新之前调用就均分
    };

    memcpy(predictor->kf_left.ControlVector, u_left, sizeof(u_left));
    memcpy(predictor->kf_right.ControlVector, u_right, sizeof(u_right));

    // ========== 3. 获取实际测量值 ==========
    float z_left[2] = {
        chassis->left_leg.leg_angle - PI/2 - chassis->chassis_pitch,
        chassis->left_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET)
    };

    float z_right[2] = {
        chassis->right_leg.leg_angle - PI/2 - chassis->chassis_pitch,
        chassis->right_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET)
    };

    // ========== 4. 执行KF预测 ==========
    // 注意：这里需要您KF库的实际函数名
    // 假设您的KF库调用顺序是：设置状态 -> 预测 -> 更新

    // 左腿KF
    memcpy(predictor->kf_left.MeasuredVector, z_left, sizeof(z_left));
    // Kalman_Filter_Predict(&predictor->kf_left);     // 预测步
    Kalman_Filter_Update(&predictor->kf_left);        // 更新步

    // 右腿KF
    memcpy(predictor->kf_right.MeasuredVector, z_right, sizeof(z_right));
    // Kalman_Filter_Predict(&predictor->kf_right);    // 预测步
    Kalman_Filter_Update(&predictor->kf_right);       // 更新步

    // ========== 5. 获取预测结果 ==========
    predictor->theta_pred_left = predictor->kf_left.Output[0];   // 角速度是第二个状态
    predictor->theta_pred_right = predictor->kf_right.Output[0];
    predictor->omega_pred_left = predictor->kf_left.Output[1];   // 角速度是第二个状态
    predictor->omega_pred_right = predictor->kf_right.Output[1];

    // ========== 6. 计算补偿力矩 ==========
    float omega_actual_left = z_left[1];
    float omega_actual_right = z_right[1];

    float omega_error_left = predictor->omega_pred_left - omega_actual_left;
    float omega_error_right = predictor->omega_pred_right - omega_actual_right;

    // 基础补偿
    predictor->comp_left = predictor->K_adjust * omega_error_left;
    predictor->comp_right = predictor->K_adjust * omega_error_right;

    // ========== 7. 柔顺滤波和限幅 ==========
    static float last_comp_left = 0.0f, last_comp_right = 0.0f;
    float alpha = 0.3f;  // 滤波系数

    predictor->comp_left = alpha * predictor->comp_left + (1.0f - alpha) * last_comp_left;
    predictor->comp_right = alpha * predictor->comp_right + (1.0f - alpha) * last_comp_right;

    last_comp_left = predictor->comp_left;
    last_comp_right = predictor->comp_right;

    // 限幅保护
    float max_comp = 3.0f;
    predictor->comp_left = fminf(fmaxf(predictor->comp_left, -max_comp), max_comp);
    predictor->comp_right = fminf(fmaxf(predictor->comp_right, -max_comp), max_comp);
}