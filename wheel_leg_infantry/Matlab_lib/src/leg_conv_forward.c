//
// Created by kun on 2026/1/19.
//

#include "leg_conv_forward.h"
#include "arm_math.h"
#include <math.h>

/**
 * @brief VMC正解算函数 - 针对STM32优化的版本
 */
/**
 * @brief VMC正解算函数 - 从关节扭矩计算摆杆支持力 (高度优化版本)
 * @param tau1, tau2 两个髋关节电机的实际扭矩 (Nm)
 * @param phi1, phi4 两个髋关节电机的角度 (rad)
 * @param F0 沿虚拟摆杆方向的力 (支持力) (N) [输出]
 * @param Tp 绕虚拟摆杆质心的力矩 (Nm) [输出]
 * @param L0 虚拟摆杆长度 (m) [输出]
 * @param phi0 虚拟摆杆角度 (rad) [输出]
 * @param F_ee 足端在基座标系下的力 [Fx; Fy] (N) [输出]
 */
void vmc_forward_stm32(float tau1, float tau2, float phi1, float phi4,
                      float* F0, float* Tp, float* L0, float* phi0,
                      float F_ee[2])
{
    // 常量
    const float l1 = 0.15f;
    const float l2 = 0.25f;
    const float l3 = 0.25f;
    const float l4 = 0.15f;
    const float l5 = 0.11f;
    const float l5_2 = 0.055f;

    // 三角函数（使用ARM优化版本）
    float cos_phi1, sin_phi1, cos_phi4, sin_phi4;
    sin_phi1 = arm_sin_f32(phi1);
    cos_phi1 = arm_cos_f32(phi1);
    sin_phi4 = arm_sin_f32(phi4);
    cos_phi4 = arm_cos_f32(phi4);

    // B点和D点坐标
    float x_B = l1 * cos_phi1;
    float y_B = l1 * sin_phi1;
    float x_D = l5 + l4 * cos_phi4;
    float y_D = l4 * sin_phi4;

    // BD向量
    float dx = x_D - x_B;
    float dy = y_D - y_B;
    float d_sq = dx * dx + dy * dy;
    float d = sqrtf(d_sq);

    // 计算被动关节和C点坐标
    float phi2, phi3, x_C, y_C;
    float cos_phi2, sin_phi2, cos_phi3, sin_phi3;

    if (d > 0.49f || d < 0.001f) { // l2+l3=0.5, |l2-l3|=0.0
        // 接近奇异，使用线性插值
        x_C = 0.5f * (x_B + x_D);
        y_C = 0.5f * (y_B + y_D) - 0.1f; // 向下偏移
        phi2 = atan2f(y_C - y_B, x_C - x_B);
        phi3 = atan2f(y_D - y_C, x_D - x_C);
    } else {
        // 正常计算
        float a = (0.0625f - 0.0625f + d_sq) / (2.0f * d); // l2^2 - l3^2 = 0
        float h_sq = 0.0625f - a * a; // l2^2 = 0.0625
        float h = sqrtf(fmaxf(h_sq, 0.0f));

        float x_P = x_B + a * dx / d;
        float y_P = y_B + a * dy / d;

        // 选择下方解
        x_C = x_P + h * dy / d;
        y_C = y_P - h * dx / d;

        phi2 = atan2f(y_C - y_B, x_C - x_B);
        phi3 = atan2f(y_D - y_C, x_D - x_C);
    }

    // 计算摆杆参数
    float dx_C = x_C - l5_2;
    float L0_sq = dx_C * dx_C + y_C * y_C;
    *L0 = sqrtf(L0_sq);
    *phi0 = atan2f(y_C, dx_C);

    // 计算雅可比矩阵元素
    sin_phi2 = arm_sin_f32(phi2);
    cos_phi2 = arm_cos_f32(phi2);
    sin_phi3 = arm_sin_f32(phi3);
    cos_phi3 = arm_cos_f32(phi3);

    float delta23 = phi2 - phi3;
    float delta12 = phi1 - phi2;
    float delta34 = phi3 - phi4;

    float sin_delta23 = arm_sin_f32(delta23);
    if (fabsf(sin_delta23) < 1e-6f) {
        sin_delta23 = 1e-6f;
    }

    float sin_delta12 = arm_sin_f32(delta12);
    float sin_delta34 = arm_sin_f32(delta34);

    // 雅可比矩阵
    float J11 = (0.15f * sin_phi3 * sin_delta12) / sin_delta23;
    float J12 = (0.15f * sin_phi2 * sin_delta34) / sin_delta23;
    float J21 = -(0.15f * cos_phi3 * sin_delta12) / sin_delta23;
    float J22 = -(0.15f * cos_phi2 * sin_delta34) / sin_delta23;

    // 直接求解 F_ee = (Jᵀ)^-1 * τ
    float det = J11 * J22 - J21 * J12;

    if (fabsf(det) > 1e-6f) {
        float inv_det = 1.0f / det;
        F_ee[0] = (J22 * tau1 - J21 * tau2) * inv_det;
        F_ee[1] = (-J12 * tau1 + J11 * tau2) * inv_det;
    } else {
        // 奇异位置，使用伪逆简化
        float norm_sq = J11 * J11 + J12 * J12 + J21 * J21 + J22 * J22;
        if (norm_sq > 1e-12f) {
            float scale = 1.0f / norm_sq;
            F_ee[0] = (J11 * tau1 + J21 * tau2) * scale;
            F_ee[1] = (J12 * tau1 + J22 * tau2) * scale;
        } else {
            F_ee[0] = 0.0f;
            F_ee[1] = 0.0f;
        }
    }

    // 转换到摆杆坐标系
    float cos_phi0 = arm_cos_f32(*phi0);
    float sin_phi0 = arm_sin_f32(*phi0);

    float F_local_x = cos_phi0 * F_ee[0] + sin_phi0 * F_ee[1];
    float F_local_y = -sin_phi0 * F_ee[0] + cos_phi0 * F_ee[1];

    // 输出结果
    *F0 = F_local_x;
    *Tp = F_local_y * (*L0 * 0.5f);
}