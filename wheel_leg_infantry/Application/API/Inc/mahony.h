//
// Created by kun on 2025/11/27.
//

#ifndef WHEEL_LEG_INFANTRY_MAHONY_H
#define WHEEL_LEG_INFANTRY_MAHONY_H
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float q[4];      // 四元数（w, x, y, z）
    float EulerAngle[3];
    float twoKp;               // 2 * proportional gain
    float twoKi;               // 2 * integral gain
    float integralFBx;
    float integralFBy;
    float integralFBz;         // 积分反馈分量
} MahonyAHRS;
extern void MahonyAHRS_Init(MahonyAHRS *ahrs, float kp, float ki);
extern void MahonyAHRS_Update(MahonyAHRS *ahrs,
                                     float g[],
                                     float a[],
                                     float m[],
                                     float dt);
#endif //WHEEL_LEG_INFANTRY_MAHONY_H