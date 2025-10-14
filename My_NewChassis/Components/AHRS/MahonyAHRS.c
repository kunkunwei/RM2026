//=============================================================================================
// MahonyAHRS.c (C版本)
//=============================================================================================

#include "AHRS.h"
#include <math.h>

#define DEFAULT_SAMPLE_FREQ 512.0f // 默认采样频率 Hz
#define twoKpDef (2.0f * 0.5f)    // 2 * 比例增益
#define twoKiDef (2.0f * 0.0f)    // 2 * 积分增益

static float MahonyAHRS_invSqrt(float x)
{
    float halfx = 0.5f * x;
    union { float f; long l; } i;
    i.f = x;
    i.l = 0x5f3759df - (i.l >> 1);
    float y = i.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void MahonyAHRS_ComputeAngles(MahonyAHRS_t *mahony)
{
    mahony->roll = atan2f(mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3, 0.5f - mahony->q1 * mahony->q1 - mahony->q2 * mahony->q2);
    mahony->pitch = asinf(-2.0f * (mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2));
    mahony->yaw = atan2f(mahony->q1 * mahony->q2 + mahony->q0 * mahony->q3, 0.5f - mahony->q2 * mahony->q2 - mahony->q3 * mahony->q3);
    mahony->anglesComputed = 1;
}

void MahonyAHRS_Init(MahonyAHRS_t *mahony, float sampleFrequency)
{
    mahony->twoKp = twoKpDef;
    mahony->twoKi = twoKiDef;
    mahony->q0 = 1.0f;
    mahony->q1 = 0.0f;
    mahony->q2 = 0.0f;
    mahony->q3 = 0.0f;
    mahony->integralFBx = 0.0f;
    mahony->integralFBy = 0.0f;
    mahony->integralFBz = 0.0f;
    mahony->anglesComputed = 0;
    mahony->invSampleFreq = 1.0f / sampleFrequency;
}

void MahonyAHRS_Update(MahonyAHRS_t *mahony, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果磁力计无效，使用IMU算法
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRS_UpdateIMU(mahony, gx, gy, gz, ax, ay, az);
        return;
    }

    // 陀螺仪单位转换
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // 加速度计有效才计算反馈
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 加速度计归一化
        recipNorm = MahonyAHRS_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // 磁力计归一化
        recipNorm = MahonyAHRS_invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        // 辅助变量
        q0q0 = mahony->q0 * mahony->q0;
        q0q1 = mahony->q0 * mahony->q1;
        q0q2 = mahony->q0 * mahony->q2;
        q0q3 = mahony->q0 * mahony->q3;
        q1q1 = mahony->q1 * mahony->q1;
        q1q2 = mahony->q1 * mahony->q2;
        q1q3 = mahony->q1 * mahony->q3;
        q2q2 = mahony->q2 * mahony->q2;
        q2q3 = mahony->q2 * mahony->q3;
        q3q3 = mahony->q3 * mahony->q3;
        // 地磁方向
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        // 估算重力和磁场方向
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        // 误差项
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        // 积分反馈
        if (mahony->twoKi > 0.0f) {
            mahony->integralFBx += mahony->twoKi * halfex * mahony->invSampleFreq;
            mahony->integralFBy += mahony->twoKi * halfey * mahony->invSampleFreq;
            mahony->integralFBz += mahony->twoKi * halfez * mahony->invSampleFreq;
            gx += mahony->integralFBx;
            gy += mahony->integralFBy;
            gz += mahony->integralFBz;
        } else {
            mahony->integralFBx = 0.0f;
            mahony->integralFBy = 0.0f;
            mahony->integralFBz = 0.0f;
        }
        // 比例反馈
        gx += mahony->twoKp * halfex;
        gy += mahony->twoKp * halfey;
        gz += mahony->twoKp * halfez;
    }
    // 四元数积分
    gx *= (0.5f * mahony->invSampleFreq);
    gy *= (0.5f * mahony->invSampleFreq);
    gz *= (0.5f * mahony->invSampleFreq);
    qa = mahony->q0;
    qb = mahony->q1;
    qc = mahony->q2;
    mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
    mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
    mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
    mahony->q3 += (qa * gz + qb * gy - qc * gx);
    // 四元数归一化
    recipNorm = MahonyAHRS_invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= recipNorm;
    mahony->q1 *= recipNorm;
    mahony->q2 *= recipNorm;
    mahony->q3 *= recipNorm;
    mahony->anglesComputed = 0;
}

void MahonyAHRS_UpdateIMU(MahonyAHRS_t *mahony, float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    // 陀螺仪单位转换
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    // 加速度计有效才计算反馈
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 加速度计归一化
        recipNorm = MahonyAHRS_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // 估算重力方向
        halfvx = mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2;
        halfvy = mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3;
        halfvz = mahony->q0 * mahony->q0 - 0.5f + mahony->q3 * mahony->q3;
        // 误差项
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        // 积分反馈
        if (mahony->twoKi > 0.0f) {
            mahony->integralFBx += mahony->twoKi * halfex * mahony->invSampleFreq;
            mahony->integralFBy += mahony->twoKi * halfey * mahony->invSampleFreq;
            mahony->integralFBz += mahony->twoKi * halfez * mahony->invSampleFreq;
            gx += mahony->integralFBx;
            gy += mahony->integralFBy;
            gz += mahony->integralFBz;
        } else {
            mahony->integralFBx = 0.0f;
            mahony->integralFBy = 0.0f;
            mahony->integralFBz = 0.0f;
        }
        // 比例反馈
        gx += mahony->twoKp * halfex;
        gy += mahony->twoKp * halfey;
        gz += mahony->twoKp * halfez;
    }
    // 四元数积分
    gx *= (0.5f * mahony->invSampleFreq);
    gy *= (0.5f * mahony->invSampleFreq);
    gz *= (0.5f * mahony->invSampleFreq);
    qa = mahony->q0;
    qb = mahony->q1;
    qc = mahony->q2;
    mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
    mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
    mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
    mahony->q3 += (qa * gz + qb * gy - qc * gx);
    // 四元数归一化
    recipNorm = MahonyAHRS_invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= recipNorm;
    mahony->q1 *= recipNorm;
    mahony->q2 *= recipNorm;
    mahony->q3 *= recipNorm;
    mahony->anglesComputed = 0;
}

// 四元数初始化
void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    // 简单初始化：假设初始姿态为加速度方向，忽略磁力计
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

// 四元数更新（带磁力计）
bool_t AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3])
{
    MahonyAHRS_t mahony;
    MahonyAHRS_Init(&mahony, 1.0f / timing_time);
    mahony.q0 = quat[0];
    mahony.q1 = quat[1];
    mahony.q2 = quat[2];
    mahony.q3 = quat[3];
    MahonyAHRS_Update(&mahony, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
    quat[0] = mahony.q0;
    quat[1] = mahony.q1;
    quat[2] = mahony.q2;
    quat[3] = mahony.q3;
    return 1;
}

// 获取欧拉角 yaw
fp32 get_yaw(const fp32 quat[4])
{
    MahonyAHRS_t mahony;
    mahony.q0 = quat[0];
    mahony.q1 = quat[1];
    mahony.q2 = quat[2];
    mahony.q3 = quat[3];
    return MahonyAHRS_GetYawRad(&mahony);
}

// 获取欧拉角 pitch
fp32 get_pitch(const fp32 quat[4])
{
    MahonyAHRS_t mahony;
    mahony.q0 = quat[0];
    mahony.q1 = quat[1];
    mahony.q2 = quat[2];
    mahony.q3 = quat[3];
    return MahonyAHRS_GetPitchRad(&mahony);
}

// 获取欧拉角 roll
fp32 get_roll(const fp32 quat[4])
{
    MahonyAHRS_t mahony;
    mahony.q0 = quat[0];
    mahony.q1 = quat[1];
    mahony.q2 = quat[2];
    mahony.q3 = quat[3];
    return MahonyAHRS_GetRollRad(&mahony);
}

// 获取欧拉角 yaw, pitch, roll
void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    MahonyAHRS_t mahony;
    mahony.q0 = quat[0];
    mahony.q1 = quat[1];
    mahony.q2 = quat[2];
    mahony.q3 = quat[3];
    *yaw = MahonyAHRS_GetYawRad(&mahony);
    *pitch = MahonyAHRS_GetPitchRad(&mahony);
    *roll = MahonyAHRS_GetRollRad(&mahony);
}

// 获取重力加速度
fp32 get_carrier_gravity(void)
{
    return 9.80665f;
}
