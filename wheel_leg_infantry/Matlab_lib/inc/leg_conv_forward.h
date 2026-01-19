//
// Created by kun on 2026/1/19.
//

#ifndef __LEG_CONV_FORWARD_H
#define __LEG_CONV_FORWARD_H
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
                      float F_ee[2]);
#endif //WHEEL_LEG_INFANTRY_LEG_CONV_FORWARD_H