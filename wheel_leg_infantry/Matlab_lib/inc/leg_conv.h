/*
 * File: leg_conv.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Jul-2024 18:43:29
 */

#ifndef LEG_CONV_H
#define LEG_CONV_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
/**
 * @brief 将足端期望的力和力矩转换为两个驱动电机的目标力矩（雅可比转置）
 *
 * @param F 足端沿腿长方向的力
 * @param Tp 足端垂直于腿长方向的力（产生力矩）
 * @param phi1 电机1（髋关节）的角度
 * @param phi4 电机2（髋关节）的角度
 * @param T 输出的两个电机的力矩，T[0]为电机1力矩，T[1]为电机2力矩
 */
extern void leg_conv(float F, float Tp, float phi1, float phi4, float T[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_conv.h
 *
 * [EOF]
 */
