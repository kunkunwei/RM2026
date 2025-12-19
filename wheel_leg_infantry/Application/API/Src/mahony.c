//
// Created by kun on 2025/11/27.
//

#include "mahony.h"
#include "math.h"
#include "pid.h"
#include "config.h"
/* 初始化
 * kp, ki: 推荐 kp = 0.5 ~ 5.0, ki = 0.0 ~ 0.1 （视系统噪声和采样率调整）
 * 若 ki==0 则禁用积分项
 */
 void MahonyAHRS_Init(MahonyAHRS *ahrs, float kp, float ki)
{
    ahrs->q[0] = 1.0f;
    ahrs->q[1] = 0.0f;
    ahrs->q[2] = 0.0f;
    ahrs->q[3] = 0.0f;
    ahrs->twoKp = 2.0f * kp;
    ahrs->twoKi = 2.0f * ki;
    ahrs->integralFBx = 0.0f;
    ahrs->integralFBy = 0.0f;
    ahrs->integralFBz = 0.0f;
}

/* 内部工具：安全的倒开方（使用标准 sqrtf） */
static inline float invSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    return 1.0f / sqrtf(x);
}

/* 更新函数：融合加速度计、陀螺仪、磁力计
 * g[0],g[1],g[2] - 陀螺：rad/s
 * a[0],a[1],a[2] - 加速度：任意相对单位（会归一化）
 * m[0],m[1],m[2] - 磁力计：任意相对单位（会归一化）
 * dt - 秒
 */
 void MahonyAHRS_Update(MahonyAHRS *ahrs,
                                     float g[],
                                     float a[], 
                                     float m[],
                                     float dt)
{
    float q0 = ahrs->q[0], q1 = ahrs->q[1], q2 = ahrs->q[2], q3 = ahrs->q[3];
    float recipNorm;
    float hx, hy, bx, bz;
    float vx, vy, vz;
    float wx, wy, wz;
    float ex, ey, ez;
    float pa, pb, pc;

    // 1) 归一化加速度测量
    recipNorm = invSqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
    if (recipNorm == 0.0f) return; // 无效数据，跳过
    a[0] *= recipNorm;
    a[1] *= recipNorm;
    a[2] *= recipNorm;

    // 2) 归一化磁力计测量
    recipNorm = invSqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    if (recipNorm == 0.0f) {
        // 如果磁力计无效，可使用纯 IMU 更新的简化 Mahony（这里我们直接返回，不更新）
        return;
    }
    m[0] *= recipNorm;
    m[1] *= recipNorm;
    m[2] *= recipNorm;

    // 3) 参考系旋转：估计地磁方向（参考 Mahony 原始式子）
    // 由四元数计算地磁在机体坐标系的投影
    // hx, hy 是辅助变量，bx 是地磁在机体x方向的水平分量，bz垂直分量
    // 公式来自 Mahony paper / Madgwick 衍生
    hx = 2.0f * (m[0] * (0.5f - q2*q2 - q3*q3) + m[1] * (q1*q2 - q0*q3) + m[2] * (q1*q3 + q0*q2));
    hy = 2.0f * (m[0] * (q1*q2 + q0*q3) + m[1] * (0.5f - q1*q1 - q3*q3) + m[2] * (q2*q3 - q0*q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (m[0] * (q1*q3 - q0*q2) + m[1] * (q2*q3 + q0*q1) + m[2] * (0.5f - q1*q1 - q2*q2));

    // 4) 估计方向向量（重力方向和地磁方向由四元数计算）
    // 重力方向估计
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 磁矢量估计（Earth magnetic field components in body frame）
    wx = 2.0f * bx * (0.5f - q2*q2 - q3*q3) + 2.0f * bz * (q1*q3 - q0*q2);
    wy = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (q0*q1 + q2*q3);
    wz = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1*q1 - q2*q2);

    // 5) 误差：测量方向与估计方向的叉乘（加速度与磁力的组合误差）
    ex = (a[1] * vz - a[2] * vy) + (m[1] * wz - m[2] * wy);
    ey = (a[2] * vx - a[0] * vz) + (m[2] * wx - m[0] * wz);
    ez = (a[0] * vy - a[1] * vx) + (m[0] * wy - m[1] * wx);

    // 6) 积分反馈（防止陀螺零偏）
    if (ahrs->twoKi > 0.0f) {
        ahrs->integralFBx += ahrs->twoKi * ex * dt;
        ahrs->integralFBy += ahrs->twoKi * ey * dt;
        ahrs->integralFBz += ahrs->twoKi * ez * dt;
        // 对积分项做一定限幅以防风火轮（可自行调整）
        const float integratorLimit = 0.5f; // 经验值
        if (ahrs->integralFBx > integratorLimit) ahrs->integralFBx = integratorLimit;
        if (ahrs->integralFBx < -integratorLimit) ahrs->integralFBx = -integratorLimit;
        if (ahrs->integralFBy > integratorLimit) ahrs->integralFBy = integratorLimit;
        if (ahrs->integralFBy < -integratorLimit) ahrs->integralFBy = -integratorLimit;
        if (ahrs->integralFBz > integratorLimit) ahrs->integralFBz = integratorLimit;
        if (ahrs->integralFBz < -integratorLimit) ahrs->integralFBz = -integratorLimit;
    } else {
        // 如果禁用积分则清零，避免遗留误差
        ahrs->integralFBx = 0.0f;
        ahrs->integralFBy = 0.0f;
        ahrs->integralFBz = 0.0f;
    }

    // 7) 将反馈加到陀螺角速度上（比例 + 积分）
    g[0] += ahrs->twoKp * ex + ahrs->integralFBx;
    g[1] += ahrs->twoKp * ey + ahrs->integralFBy;
    g[2] += ahrs->twoKp * ez + ahrs->integralFBz;

    // 8) 四元数积分（q_dot = 0.5 * q * omega）
    pa = q1;
    pb = q2;
    pc = q3;
    q0 += (-pa * g[0] - pb * g[1] - pc * g[2]) * (0.5f * dt);
    q1 += ( q0 * g[0] + pb * g[2] - pc * g[1]) * (0.5f * dt);
    q2 += ( q0 * g[1] - pa * g[2] + pc * g[0]) * (0.5f * dt);
    q3 += ( q0 * g[2] + pa * g[1] - pb * g[0]) * (0.5f * dt);

    // 9) 归一化四元数
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    ahrs->q[0] = q0 * recipNorm;
    ahrs->q[1] = q1 * recipNorm;
    ahrs->q[2] = q2 * recipNorm;
    ahrs->q[3] = q3 * recipNorm;
}

/* 从四元数获取欧拉角（弧度）
 * 返回 roll, pitch, yaw （右手系，yaw 为北向角）
 * 注意：返回的 yaw 直接由四元数导出，通常需要加上磁偏角 declination 才是真正的航向。
 */
static inline void MahonyAHRS_GetEuler(const MahonyAHRS *ahrs, float *roll, float *pitch, float *yaw)
{
    float q0 = ahrs->q[0];
    float q1 = ahrs->q[1];
    float q2 = ahrs->q[2];
    float q3 = ahrs->q[3];

    // roll (x-axis rotation)
    *roll = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));

    // pitch (y-axis rotation)
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp >= 1.0f)
        *pitch = M_PI / 2.0f; // use 90 degrees if out of range
    else if (sinp <= -1.0f)
        *pitch = -M_PI / 2.0f;
    else
        *pitch = asinf(sinp);

    // yaw (z-axis rotation)
    *yaw = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
}

/* 获取四元数（方便直接读取） */
 void MahonyAHRS_GetQuaternion(const MahonyAHRS *ahrs, float *q0, float *q1, float *q2, float *q3)
{
    *q0 = ahrs->q[0];
    *q1 = ahrs->q[1];
    *q2 = ahrs->q[2];
    *q3 = ahrs->q[3];
}