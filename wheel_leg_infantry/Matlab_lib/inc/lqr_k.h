/*
 * File: lqr_k.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Jul-2024 13:44:59
 */

#ifndef LQR_K_H
#define LQR_K_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "arm_math.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
/**
 * @brief 根据期望腿长L0计算LQR增益矩阵K（增益调度）
 *
 * @param L0 期望的腿长
 * @param K 输出的LQR增益矩阵（1x12或2x6）
 */
extern void lqr_k(float L0, float K[12]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for lqr_k.h
 *
 * [EOF]
 */
