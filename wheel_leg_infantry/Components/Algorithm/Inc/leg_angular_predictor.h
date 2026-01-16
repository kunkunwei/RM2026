// leg_angular_predictor.h
#ifndef LEG_ANGULAR_PREDICTOR_H
#define LEG_ANGULAR_PREDICTOR_H

#include "chassis_task.h"
#include "main.h"
#include "kalman.h"

#ifdef __cplusplus
extern "C" {
#endif
/*------------WBR系统-----------------
    *         机体 M, φ (俯仰角)
               ↑
               │ L (虚拟摆杆长度)
    虚拟摆杆 m, θ (与竖直方向夹角)
               │
            驱动轮, x (水平位移)
 */
/* ==================== 配置参数 ==================== */
#define PREDICTOR_DT          0.005f     // 预测周期 5ms
#define LEG_MASS_EFFECTIVE    7.5f       // 单腿等效质量 [kg]
#define WHEEL_RADIUS          0.08f      // 轮子半径 [m]

/* ==================== 数据结构 ==================== */

/**
 * @brief 双腿角速度预测器（简化版）
 */
typedef struct {
    // 左右腿卡尔曼滤波器
    KalmanFilter_Info_TypeDef kf_left;
    KalmanFilter_Info_TypeDef kf_right;

    // 当前预测输出
    float theta_pred_left;     // 左腿预测摆杆角度 [rad]
    float theta_pred_right;    // 右腿预测摆杆角度 [rad]
    float omega_pred_left;      // 左腿预测角速度 [rad/s]
    float omega_pred_right;     // 右腿预测角速度 [rad/s]
    float comp_left;           // 左腿补偿力矩 [N·m]
    float comp_right;          // 右腿补偿力矩 [N·m]

    // 配置参数
    float K_adjust;            // 自适应补偿增益

} LegPredictor_t;

/* ==================== 函数接口（仅3个！） ==================== */

/**
 * @brief 初始化预测器（一次性调用）
 * @param predictor 预测器指针
 * @param chassis 底盘状态指针（用于获取初始状态）
 * @param K_adjust 补偿增益
 */
void LegPredictor_Init(LegPredictor_t *predictor,
                      const chassis_move_t *chassis,
                      float K_adjust);

/**
 * @brief 主更新函数（每个控制周期调用）
 * @param predictor 预测器指针
 * @param chassis 底盘状态指针
 * @param leg_torque 总关节力矩 [N·m]
 * @param wheel_torque 总轮子力矩 [N·m]
 * @param err_torque 双腿误差力矩 [N·m]
 */
void LegPredictor_Update(LegPredictor_t *predictor,
                        const chassis_move_t *chassis,
                        float leg_torque,
                        float wheel_torque,
                        float err_torque);

// /**
//  * @brief 获取补偿力矩（供控制循环使用）
//  * @param predictor 预测器指针
//  * @param comp_left 输出：左腿补偿力矩
//  * @param comp_right 输出：右腿补偿力矩
//  */
// static inline void LegPredictor_GetCompensation(const LegPredictor_t *predictor,
//                                                float *comp_left,
//                                                float *comp_right)
// {
//     if (predictor && comp_left && comp_right) {
//         *comp_left = predictor->comp_left;
//         *comp_right = predictor->comp_right;
//     }
// }

#ifdef __cplusplus
}
#endif

#endif // LEG_ANGULAR_PREDICTOR_H