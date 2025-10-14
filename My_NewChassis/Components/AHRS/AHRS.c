//=============================================================================================
// AHRS.c (C版本)
//=============================================================================================

#include "AHRS.h"
#include <math.h>

// Mahony算法参数
#define DEFAULT_SAMPLE_FREQ 512.0f
#define twoKpDef (2.0f * 0.5f)
#define twoKiDef (2.0f * 0.0f)

// 内部结构体
typedef struct {
    fp32 twoKp;
    fp32 twoKi;
    fp32 q0, q1, q2, q3;
    fp32 integralFBx, integralFBy, integralFBz;
    fp32 invSampleFreq;
} MahonyAHRS_t;

// 内部快速开方
static fp32 MahonyAHRS_invSqrt(fp32 x)
{
    fp32 halfx = 0.5f * x;
    union { fp32 f; long l; } i;
    i.f = x;
    i.l = 0x5f3759df - (i.l >> 1);
    fp32 y = i.f;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// 内部初始化
static void MahonyAHRS_Init(MahonyAHRS_t *mahony, fp32 sampleFrequency)
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
    mahony->invSampleFreq = 1.0f / sampleFrequency;
}

// 内部更新（带磁力计）
static void MahonyAHRS_Update(MahonyAHRS_t *mahony, fp32 gx, fp32 gy, fp32 gz, fp32 ax, fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz)
{
    fp32 recipNorm;
    fp32 q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    fp32 hx, hy, bx, bz;
    fp32 halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    fp32 halfex, halfey, halfez;

    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        // 无磁力计，调用IMU版本
        goto IMU_ONLY;
    }

    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = MahonyAHRS_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        recipNorm = MahonyAHRS_invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
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
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
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
    goto QUAT_UPDATE;

IMU_ONLY:
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = MahonyAHRS_invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        fp32 halfvx = mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2;
        fp32 halfvy = mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3;
        fp32 halfvz = mahony->q0 * mahony->q0 - 0.5f + mahony->q3 * mahony->q3;
        fp32 halfex = (ay * halfvz - az * halfvy);
        fp32 halfey = (az * halfvx - ax * halfvz);
        fp32 halfez = (ax * halfvy - ay * halfvx);
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

QUAT_UPDATE:
    fp32 qa = mahony->q0;
    fp32 qb = mahony->q1;
    fp32 qc = mahony->q2;
    mahony->q0 += (-qb * gx - qc * gy - mahony->q3 * gz);
    mahony->q1 += (qa * gx + qc * gz - mahony->q3 * gy);
    mahony->q2 += (qa * gy - qb * gz + mahony->q3 * gx);
    mahony->q3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = MahonyAHRS_invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= recipNorm;
    mahony->q1 *= recipNorm;
    mahony->q2 *= recipNorm;
    mahony->q3 *= recipNorm;
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
    // 四元数转yaw（弧度）
    fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    return atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
}

// 获取欧拉角 pitch
fp32 get_pitch(const fp32 quat[4])
{
    fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    return asinf(-2.0f * (q1 * q3 - q0 * q2));
}

// 获取欧拉角 roll
fp32 get_roll(const fp32 quat[4])
{
    fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    return atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
}

// 获取欧拉角 yaw, pitch, roll
void get_angle(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = get_yaw(quat);
    *pitch = get_pitch(quat);
    *roll = get_roll(quat);
}

// 获取重力加速度
fp32 get_carrier_gravity(void)
{
    return 9.80665f;
}
