/*
 * File: leg_pos.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 03-Jul-2024 18:42:00
 */

#ifndef LEG_POS_H
#define LEG_POS_H

/* Include Files */
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
/**
 * @brief 根据电机角度计算足端位置（正运动学）
 *
 * @param phi1 电机1（髋关节）的角度
 * @param phi4 电机2（膝关节）的角度
 * @param pos 输出的足端位置（极坐标），pos[0]为腿长，pos[1]为腿部角度
 */
extern void leg_pos(float phi1, float phi4, float pos[2]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_pos.h
 *
 * [EOF]
 */
