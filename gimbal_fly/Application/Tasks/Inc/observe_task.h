/**
  ******************************************************************************
  * @file           : observe_task.h
  * @brief          : 观测任务头文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 实现轮足机器人的状态观测和速度估计功能
  *                  使用卡尔曼滤波进行传感器融合，提高运动状态估计精度
  ******************************************************************************
  */

#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "main.h"
#include "stdio.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "INS_Task.h"

#include "remote_control.h"

#include "kalman.h"

/* 观测任务函数声明 */
extern void ObserveTask(void const * argument);

/* 卡尔曼滤波器相关函数 */
extern void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel);

/* 速度获取接口函数 */
extern fp32 get_KF_Spd(void);    // 获取卡尔曼滤波后的速度
extern fp32 get_raw_Spd(void);   // 获取原始平均速度
extern fp32 get_diff_Spd(void);  // 获取差速(用于检测打滑)
extern fp32 get_body_Spd(void);  // 获取机体速度

#endif




