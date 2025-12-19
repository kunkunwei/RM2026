/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : slip_detector.h
 * @brief          : Slip Detection and Kalman Filter Optimization Module
 * @author         : AI Assistant
 * @date           : 2025/11/29
 * @version        : v1.0
 ******************************************************************************
 * @attention      : 多特征融合的打滑检测，动态调整卡尔曼滤波器观测噪声
 *                   支持Innovation Test（残差检验）
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef SLIP_DETECTOR_H
#define SLIP_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
// #include "chassis_task.h"
#include "config.h"
// struct chassis_move_t;
/* ==================== 配置宏定义 ==================== */
    // 特征阈值（可根据实际调试调整）
#define SLIP_DIFF_THRESHOLD        0.8f   // 差速异常阈值 (m/s)
#define SLIP_VEL_ERROR_THRESHOLD   0.5f   // 速度误差阈值 (m/s)
#define SLIP_ACCEL_THRESHOLD       2.0f   // 加速度异常阈值 (m/s²)
#define SLIP_INNOVATION_THRESHOLD  0.4f   // 卡尔曼残差阈值 (m/s)

    // 特征权重（总和建议为1.0，突出关键特征）
#define SLIP_WEIGHT_DIFF           0.25f  // 差速特征权重
#define SLIP_WEIGHT_VEL            0.25f  // 速度误差权重
#define SLIP_WEIGHT_ACCEL          0.3f   // 加速度特征权重（打滑时影响更大）
#define SLIP_WEIGHT_INNOVATION     0.2f   // 残差特征权重

    // 状态判定参数
#define SLIP_CONFIDENCE_ENTER      0.63f   // 进入打滑的置信度阈值
#define SLIP_CONFIDENCE_EXIT       0.3f   // 退出打滑的置信度阈值
#define SLIP_ENTER_COUNT           5      // 连续n次满足则判定为打滑
#define SLIP_EXIT_COUNT            8      // 连续n次满足则判定为正常

    // 噪声缩放系数
#define R_SCALE_MIN                1.0f   // 最小观测噪声系数
#define R_SCALE_MAX                20.0f   // 最大观测噪声系数（打滑时增大）
#define WHEEL_BASE_HALF            0.15f  // 轮距一半 (m)，与底盘参数一致

    //更新配置宏定义

/* ==================== 数据结构 ==================== */
    /**
     * @brief 打滑状态枚举（支持单轮/双轮区分）
     * 注：使用位运算支持组合状态（如SLIP_LEFT | SLIP_RIGHT表示双轮打滑）
     */
    typedef enum {
        SLIP_NONE    = 0,  // 无打滑
        SLIP_LEFT    = 1,  // 左轮打滑
        SLIP_RIGHT   = 2,  // 右轮打滑
        SLIP_BOTH    = 3   // 双轮打滑
    } SlipFlag;

    /**
     * @brief 单轮打滑特征参数
     */
    typedef struct {
        float diff;           // 差速特征值（0~1）
        float vel_error;      // 速度误差特征值（0~1）
        float accel_anomaly;  // 加速度异常特征值（0~1）
        float innovation;     // 卡尔曼残差特征值（0~1）
        float confidence;     // 单轮打滑置信度（0~1）
        int slip_counter;     // 打滑计数（防抖）
        int normal_counter;   // 正常计数（防抖）
        // 新增：特征值历史缓存（用于滑窗滤波）
        float diff_history[8];
        float vel_error_history[8];
        float accel_anomaly_history[8];
        float innovation_history[8];
        uint8_t history_idx;  // 缓存索引
        float torque_buffer[8];  // 缓存5个历史值（窗口大小5）
        uint8_t torque_buf_idx;      // 缓存索引
    } WheelSlipFeature;

    /**
     * @brief 打滑检测器结构体
     */
    typedef struct {
        // 输入数据（左右轮独立）
        float wheel_vel_left;   // 左轮速度 (m/s)
        float wheel_vel_right;  // 右轮速度 (m/s)
        float kf_velocity;      // 卡尔曼滤波估计速度 (m/s)
        float imu_accel_x;      // IMU X轴加速度 (m/s²)
        float left_torque_cmd;  // 左轮扭矩指令 (N·m)
        float right_torque_cmd; // 右轮扭矩指令 (N·m)
        float yaw_rate_cmd;     // 期望偏航角速度 (rad/s)

        // 左右轮打滑特征
        WheelSlipFeature left;
        WheelSlipFeature right;

        // 全局状态
        SlipFlag slip_flag;     // 组合打滑状态（位运算结果）
        float r_scale_velocity; // 速度观测噪声缩放系数
        float r_scale_accel;    // 加速度观测噪声缩放系数
    } SlipDetector_t;

/* ==================== 函数接口 ==================== */
extern void SlipDetector_Init(SlipDetector_t *detector);
extern void SlipDetector_Update(SlipDetector_t *detector) ;
extern void SlipDetector_SyncData(SlipDetector_t *detector, const float left_wheel_speed, const float right_wheel_speed,
        const float imu_accel_x,const float left_wheel_current_cmd,const float right_wheel_current_cmd,const float wz_set) ;
/**
 * @brief 获取打滑标志
 */
extern SlipFlag SlipDetector_GetFlag(const SlipDetector_t *detector) ;

/**
 * @brief 获取左右轮打滑置信度
 */
extern void SlipDetector_GetConfidence(const SlipDetector_t *detector, float *left_conf, float *right_conf) ;

/**
 * @brief 获取观测噪声缩放系数
 */
extern void SlipDetector_GetRScale(const SlipDetector_t *detector, float *r_vel, float *r_accel) ;
#ifdef __cplusplus
}
#endif

#endif // SLIP_DETECTOR_H
