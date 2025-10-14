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

extern void ObserveTask(void const * argument);
extern void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel);

extern fp32 get_KF_Spd(void);
extern fp32 get_raw_Spd(void);
extern fp32 get_diff_Spd(void);
extern fp32 get_body_Spd(void);
#endif




