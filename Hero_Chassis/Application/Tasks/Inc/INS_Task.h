/* 用户代码开始 */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : 惯性导航系统任务
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 无
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 包含惯性导航系统信息的结构体类型定义
 */
typedef struct 
{
	float pit_angle;
	float yaw_angle;
	float yaw_tolangle;
	float rol_angle;

  float pit_gyro;
  float yaw_gyro;
  float rol_gyro;

  float angle[3];
	float gyro[3];	
	float accel[3];
	
	float last_yawangle;
	int16_t YawRoundCount;
}INS_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
  * @brief 包含惯性导航系统信息的结构体
  */
extern INS_Info_Typedef INS_Info; 

const float *get_INS_angle_point();
const float *get_gyro_data_point();
const float *get_accel_data_point();
/* Exported functions prototypes ---------------------------------------------*/

#endif //INS_TASK_H
