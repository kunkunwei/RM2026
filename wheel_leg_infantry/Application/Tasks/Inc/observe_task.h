#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "INS_Task.h"
#include "chassis_task.h"
#include "remote_control.h"

#include "kalman.h"
#include "leg_angular_predictor.h"
#include "slip_detector.h"
// struct SlipFlag;
//
// #define imu_bias_estimate -0.4f  // IMU加速度偏置初始值
extern SlipDetector_t slip_detector;

extern void ObserveTask(void const * argument);
// extern void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF);
// extern void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel);

extern fp32 get_KF_Spd(void);
extern fp32 get_raw_Spd(void);
extern fp32 get_diff_Spd(void);
extern fp32 get_body_Spd(void);
extern fp32 get_imu_bias(void);

SlipDetector_t* get_slip_detector_point(void);
const LegPredictor_t *get_leg_predictor_point(void);
extern SlipFlag get_slip_flag(void);
extern fp32 get_confidence_left(void);
extern fp32 get_confidence_right(void);
extern void LegPredictor_GetCompensation( float *comp_left,float *comp_right);
#endif




