//=============================================================================================
// MahonyAHRS.h (C版本)
//=============================================================================================
// Mahony AHRS算法 C语言实现
//=============================================================================================
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// MahonyAHRS结构体
typedef struct {
    float twoKp;        // 2 * 比例增益 (Kp)
    float twoKi;        // 2 * 积分增益 (Ki)
    float q0, q1, q2, q3;   // 四元数
    float integralFBx, integralFBy, integralFBz; // 积分误差项
    float invSampleFreq;
    float roll, pitch, yaw;
    char anglesComputed;
} MahonyAHRS_t;

// 初始化
void MahonyAHRS_Init(MahonyAHRS_t *mahony, float sampleFrequency);
// 更新（带磁力计）
void MahonyAHRS_Update(MahonyAHRS_t *mahony, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
// 更新（仅IMU）
void MahonyAHRS_UpdateIMU(MahonyAHRS_t *mahony, float gx, float gy, float gz, float ax, float ay, float az);
// 获取欧拉角（角度制）
float MahonyAHRS_GetRoll(MahonyAHRS_t *mahony);
float MahonyAHRS_GetPitch(MahonyAHRS_t *mahony);
float MahonyAHRS_GetYaw(MahonyAHRS_t *mahony);
// 获取欧拉角（弧度制）
float MahonyAHRS_GetRollRad(MahonyAHRS_t *mahony);
float MahonyAHRS_GetPitchRad(MahonyAHRS_t *mahony);
float MahonyAHRS_GetYawRad(MahonyAHRS_t *mahony);

#ifdef __cplusplus
}
#endif

#endif

