/*
 * File: leg_spd.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Jul-2024 18:44:01
 */

#ifndef LEG_SPD_H
#define LEG_SPD_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
/**
 * @brief 根据电机角速度计算足端速度（正向速度运动学）
 *
 * @param dphi1 电机1（髋关节）的角速度
 * @param dphi4 电机2（膝关节）的角速度
 * @param phi1 电机1（髋关节）的角度
 * @param phi4 电机2（膝关节）的角度
 * @param spd 输出的足端速度（基于极坐标），spd[0]为径向速度（腿长变化率），spd[1]为角速度（腿部摆动角速度）
 */
extern void leg_spd(float dphi1, float dphi4, float phi1, float phi4,
                    float spd[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_spd.h
 *
 * [EOF]
 */
