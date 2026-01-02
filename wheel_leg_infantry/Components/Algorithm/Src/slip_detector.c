/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : slip_detector.c
 * @brief          : Slip Detection and Kalman Filter Optimization Implementation
 * @author         : AI Assistant
 * @date           : 2025/11/29
 * @version        : v1.0
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "slip_detector.h"
#include <stdio.h>
#include <math.h>
#include "arm_math.h"
#include "observe_task.h"
#include "chassis_task.h"

// #define WHEEL_BASE_HALF 0.27f // 轮距一半 (m)，根据实际机器人调整
/* Private functions ---------------------------------------------------------*/
/**
 * @brief 限幅到[0,1]
 */
static inline float clamp01(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < 0.0f) return 0.0f;
    return value;
}

/**
 * @brief 线性插值
 */
static inline float lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}
/**
 * @brief 滑动平均滤波
 * @param history 历史数据数组
 * @param new_val 新数据
 * @param len 数组长度
 * @param idx 当前索引（会被更新）
 * @return 滤波后的值
 */
static float sliding_avg_filter(float *history, float new_val, uint8_t len, uint8_t *idx) {
    history[*idx] = new_val;
    *idx = (*idx + 1) % len;  // 更新索引

    float sum = 0.0f;
    for (uint8_t i = 0; i < len; i++) {
        sum += history[i];
    }
    return sum / len;
}
/* Public functions ----------------------------------------------------------*/

/**
 * @brief 初始化打滑检测器
 */
void SlipDetector_Init(SlipDetector_t *detector) {
    if (detector == NULL) return;

    // 输入数据初始化
    detector->wheel_vel_left = 0.0f;
    detector->wheel_vel_right = 0.0f;
    detector->kf_velocity = 0.0f;
    detector->imu_accel_x = 0.0f;
    detector->left_torque_cmd = 0.0f;
    detector->right_torque_cmd = 0.0f;
    detector->yaw_rate_cmd = 0.0f;

    // 左右轮特征初始化
    memset(&detector->left, 0, sizeof(WheelSlipFeature));
    memset(&detector->right, 0, sizeof(WheelSlipFeature));
    // 初始化历史缓存为0
    for (int i = 0; i < 8; i++) {
        detector->left.diff_history[i] = 0.0f;
        detector->left.vel_error_history[i] = 0.0f;
        detector->left.accel_anomaly_history[i] = 0.0f;
        detector->left.innovation_history[i] = 0.0f;

        detector->right.diff_history[i] = 0.0f;
        detector->right.vel_error_history[i] = 0.0f;
        detector->right.accel_anomaly_history[i] = 0.0f;
        detector->right.innovation_history[i] = 0.0f;
    }
    for (int i = 0; i < 5; i++) {
        detector->left.torque_buffer[i] = 0.0f;
        detector->right.torque_buffer[i] = 0.0f;
    }
    detector->left.history_idx = 0;
    detector->right.history_idx = 0;

    // 全局状态初始化
    detector->slip_flag = SLIP_NONE;
    detector->r_scale_velocity = R_SCALE_MIN;
    detector->r_scale_accel = R_SCALE_MIN;
}
/**
 * @brief 计算单轮打滑特征值
 * @param wheel_vel: 轮子速度 (m/s)
 * @param other_wheel_vel: 另一个轮子速度 (m/s)
 * @param torque_cmd: 轮子扭矩指令 (N·m)
 * @param kf_vel: 卡尔曼估计速度 (m/s)
 * @param imu_accel: IMU加速度 (m/s²)
 * @param yaw_cmd: 期望偏航角速度 (rad/s)
 * @param feature: 输出特征结构体
 */
static void CalculateWheelFeature(float wheel_vel,
                                 float other_wheel_vel,
                                 float torque_cmd,
                                 float kf_vel,
                                 float imu_accel,
                                 float yaw_cmd,
                                 WheelSlipFeature *feature) {
    // 特征1：差速异常（与期望差速的偏差）
    float actual_diff = fabsf(wheel_vel - other_wheel_vel);
    float expected_diff = fabsf(yaw_cmd * 2.0f * WHEEL_BASE_HALF);
    float diff_error = fabsf(actual_diff - expected_diff);
    float raw_diff = (diff_error > SLIP_DIFF_THRESHOLD) ?
                   (diff_error / SLIP_DIFF_THRESHOLD) : 0.0f;
    raw_diff = fminf(raw_diff, 1.0f);
    // 应用滑动平均滤波
    feature->diff = sliding_avg_filter(feature->diff_history, raw_diff,
                                      5, &feature->history_idx);

    // 特征2：速度误差（与KF估计的偏差）
    float vel_error = fabsf(wheel_vel - kf_vel);
    float raw_vel_error = (vel_error > SLIP_VEL_ERROR_THRESHOLD) ?
                        (vel_error / SLIP_VEL_ERROR_THRESHOLD) : 0.0f;
    raw_vel_error = fminf(raw_vel_error, 1.0f);
    feature->vel_error = sliding_avg_filter(feature->vel_error_history, raw_vel_error,
                                           10, &feature->history_idx);

    // 特征3：加速度异常（扭矩大但加速度小）
    float wheel_inertia = 0.3f * 0.0925f;
    float expected_accel = fabsf(torque_cmd) / wheel_inertia;
    float accel_error = expected_accel - fabsf(imu_accel);
    float raw_accel = (accel_error > 0) ?
                    (accel_error / SLIP_ACCEL_THRESHOLD) : 0.0f;
    raw_accel = fminf(raw_accel, 1.0f);
    feature->accel_anomaly = sliding_avg_filter(feature->accel_anomaly_history, raw_accel,
                                               10, &feature->history_idx);

    // 特征4：卡尔曼残差
    float raw_innovation = (vel_error > SLIP_INNOVATION_THRESHOLD) ?
                         (vel_error / SLIP_INNOVATION_THRESHOLD) : 0.0f;
    raw_innovation = fminf(raw_innovation, 1.0f);
    feature->innovation = sliding_avg_filter(feature->innovation_history, raw_innovation,
                                            10, &feature->history_idx);

    // 融合置信度（保持不变）
    feature->confidence = SLIP_WEIGHT_DIFF * feature->diff +
                         SLIP_WEIGHT_VEL * feature->vel_error +
                         SLIP_WEIGHT_ACCEL * feature->accel_anomaly +
                         SLIP_WEIGHT_INNOVATION * feature->innovation;
    feature->confidence = fminf(fmaxf(feature->confidence, 0.0f), 1.0f);
}
/**
 * @brief 更新打滑检测状态
 */
void SlipDetector_Update(SlipDetector_t *detector) {
    if (detector == NULL) return;

    // 计算左右轮特征
    CalculateWheelFeature(detector->wheel_vel_left,  // 左轮速度
                         detector->wheel_vel_right, // 另一个轮子（右）
                         detector->left_torque_cmd, // 左轮扭矩
                         detector->kf_velocity,
                         detector->imu_accel_x,
                         detector->yaw_rate_cmd,
                         &detector->left);

    CalculateWheelFeature(detector->wheel_vel_right, // 右轮速度
                         detector->wheel_vel_left,  // 另一个轮子（左）
                         detector->right_torque_cmd,// 右轮扭矩
                         detector->kf_velocity,
                         detector->imu_accel_x,
                         detector->yaw_rate_cmd,
                         &detector->right);

    // 左右轮状态判定（带防抖）
    // 左轮
    if (detector->left.confidence > SLIP_CONFIDENCE_ENTER) {
        detector->left.slip_counter++;
        detector->left.normal_counter = 0;
    } else if (detector->left.confidence < SLIP_CONFIDENCE_EXIT) {
        detector->left.normal_counter++;
        detector->left.slip_counter = 0;
    }

    // 右轮
    if (detector->right.confidence > SLIP_CONFIDENCE_ENTER) {
        detector->right.slip_counter++;
        detector->right.normal_counter = 0;
    } else if (detector->right.confidence < SLIP_CONFIDENCE_EXIT) {
        detector->right.normal_counter++;
        detector->right.slip_counter = 0;
    }

    // 更新全局打滑标志（位运算组合）
    detector->slip_flag = SLIP_NONE;
    // 左轮当前是否处于打滑状态
    bool left_in_slip = (detector->left.slip_counter >= SLIP_ENTER_COUNT) &&
                       !(detector->left.normal_counter >= SLIP_EXIT_COUNT);
    // 右轮当前是否处于打滑状态
    bool right_in_slip = (detector->right.slip_counter >= SLIP_ENTER_COUNT) &&
                        !(detector->right.normal_counter >= SLIP_EXIT_COUNT);
    // 根据当前状态组合标志位
    if (left_in_slip && right_in_slip) {
        detector->slip_flag = SLIP_BOTH;
    } else if (left_in_slip) {
        detector->slip_flag = SLIP_LEFT;
    } else if (right_in_slip) {
        detector->slip_flag = SLIP_RIGHT;
    } else {
        detector->slip_flag = SLIP_NONE;
    }
    // if (detector->left.slip_counter >= SLIP_ENTER_COUNT) {
    //     detector->slip_flag |= SLIP_LEFT;
    // }
    // if (detector->right.slip_counter >= SLIP_ENTER_COUNT) {
    //     detector->slip_flag |= SLIP_RIGHT;
    // }
    // // 退出打滑判定
    // if (detector->left.normal_counter >= SLIP_EXIT_COUNT) {
    //     detector->slip_flag &= ~SLIP_LEFT;
    // }
    // if (detector->right.normal_counter >= SLIP_EXIT_COUNT) {
    //     detector->slip_flag &= ~SLIP_RIGHT;
    // }

    // 计算观测噪声缩放系数（取左右轮置信度最大值）
    float max_confidence = fmaxf(detector->left.confidence, detector->right.confidence);
    detector->r_scale_velocity = R_SCALE_MIN + (R_SCALE_MAX - R_SCALE_MIN) * max_confidence;
    detector->r_scale_accel = R_SCALE_MIN + (R_SCALE_MAX - R_SCALE_MIN) * max_confidence * 0.5f; // 加速度影响减半
}
/**
 * @brief 从底盘结构体同步数据到打滑检测器
 * @param detector: 打滑检测器结构体
 * @param chassis: 底盘控制结构体
 */
void SlipDetector_SyncData(SlipDetector_t *detector, const float left_wheel_speed, const float right_wheel_speed,
    const float imu_accel_x,const float left_wheel_current_cmd,const float right_wheel_current_cmd,const float wz_set){
    if (detector == NULL ) return;
    float imu_bias_estimate = 0.0f; // 需要从观测任务中获取IMU加速度偏置估计
    imu_bias_estimate=get_imu_bias();
    // 1. 轮速数据（单位：m/s，来自底盘反馈更新）
    detector->wheel_vel_left  =left_wheel_speed;
    detector->wheel_vel_right = right_wheel_speed;

    // 2. 卡尔曼滤波速度（来自观测任务的全局变量）
    detector->kf_velocity = get_KF_Spd();  // 假设该函数返回vaEstimateKF的速度估计

    // 3. IMU加速度（X轴）
    detector->imu_accel_x = imu_accel_x;
    // 注：imu_bias_estimate需要从observe_task.c中导出为全局变量或通过接口获取

    // 4. 轮子扭矩指令（复用LK9025_TOR_TO_CAN_DATA反向转换：电流→扭矩）
    float raw_left_torque = left_wheel_current_cmd / LK9025_TOR_TO_CAN_DATA;
    float raw_right_torque = right_wheel_current_cmd / LK9025_TOR_TO_CAN_DATA;

    // 应用滑动平均滤波
    detector->left_torque_cmd = sliding_avg_filter(
        detector->left.torque_buffer,
        raw_left_torque,
        5,
        &detector->left.torque_buf_idx
    );
    detector->right_torque_cmd = sliding_avg_filter(
        detector->right.torque_buffer,
        raw_right_torque,
        5,
        &detector->right.torque_buf_idx
    );
    // 5. 期望偏航角速度（来自底盘控制目标）
    detector->yaw_rate_cmd = wz_set;
}

/**
 * @brief 获取打滑标志
 */
SlipFlag SlipDetector_GetFlag(const SlipDetector_t *detector) {
    return (detector != NULL) ? detector->slip_flag : SLIP_NONE;
}

/**
 * @brief 获取左右轮打滑置信度
 */
void SlipDetector_GetConfidence(const SlipDetector_t *detector, float *left_conf, float *right_conf) {
    if (detector == NULL) return;
    if (left_conf != NULL) *left_conf = detector->left.confidence;
    if (right_conf != NULL) *right_conf = detector->right.confidence;
}

/**
 * @brief 获取观测噪声缩放系数
 */
void SlipDetector_GetRScale(const SlipDetector_t *detector, float *r_vel, float *r_accel) {
    if (detector == NULL) return;
    if (r_vel != NULL) *r_vel = detector->r_scale_velocity;
    if (r_accel != NULL) *r_accel = detector->r_scale_accel;
}