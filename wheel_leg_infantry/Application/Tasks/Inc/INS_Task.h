/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "api_quaternion.h"
#include "stdint.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct 
{
	float pit_angle;//扩展卡尔曼滤波后的角度，单位：度
	float yaw_angle;
	float yaw_tolangle;
	float rol_angle;

  float pit_gyro;	//扩展卡尔曼滤波后的角度速度
  float yaw_gyro;
  float rol_gyro;

  float angle[3];
	float gyro[3];	
	float accel[3];//二阶低通滤波后的数据
	
	float last_yawangle;
	int16_t YawRoundCount;
}INS_Info_Typedef;



/* Exported variables ---------------------------------------------------------*/
/**
  * @brief the structure that contains the information for the INS.
  */
extern INS_Info_Typedef INS_Info;
const float *get_INS_angle_point();
const float *get_gyro_data_point();
const float *get_accel_data_point();
const Quaternion_Info_Typedef *get_quaternion_info_point();

extern float ComputeMagYaw(const float mag[3], float roll, float pitch);
/* Exported functions prototypes ---------------------------------------------*/

#endif //INS_TASK_H

