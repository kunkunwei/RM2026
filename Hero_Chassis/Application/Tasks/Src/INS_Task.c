/* 用户代码开始 - 头文件 */
#include <stdio.h>
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : 惯性导航系统任务
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 无
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bsp_tim.h"
#include "bmi088.h"
#include "api_quaternion.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "main.h"
/**
  * @brief 包含惯性导航系统信息的结构体
  */
INS_Info_Typedef INS_Info; 

/* Private variables ---------------------------------------------------------*/
/**
  * @brief 包含二阶低通滤波器系数的数组
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
/**
  * @brief 加速度计二阶低通滤波器信息结构体
  */
LowPassFilter2p_Info_TypeDef INS_AcceLPF2p[3];  

/**
  * @brief 状态转移矩阵初始化数据
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0,
                                0, 0, 0, 1, 0, 0,
                                0, 0, 0, 0, 1, 0,
                                0, 0, 0, 0, 0, 1};
/**
  * @brief 后验协方差矩阵初始化数据
  */
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                        0.1, 0.1, 0.1, 0.1, 0.1, 100};
/**
  * @brief 温度控制PID参数初始化数据
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1600,20,0,0,0,2000};
/**
  * @brief 温度控制PID信息结构体
  */
PID_Info_TypeDef TempCtrl_PID;
/**
  * @brief 四元数信息结构体
  */
Quaternion_Info_Typedef Quaternion_Info;

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief 惯性导航系统任务初始化
 */
static void INSTask_Init(void);
/**
  * @brief BMI088温度控制
  */
static void BMI088_Temp_Control(float temp);

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief 实现惯性导航系统任务的线程函数
  * @param argument: 未使用
  * @retval 无
  */
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  TickType_t systick = 0;
	
	/* Initializes the INS_Task. */
	INSTask_Init();
	
  /* Infinite loop */
  for(;;)
  {
    systick = osKernelSysTick();
		
		/* Update the BMI088 measurement */
    BMI088_Info_Update(&BMI088_Info);

    /* accel measurement LPF2p */
    INS_Info.accel[IMU_ACCEL_INDEX_PITCH] = LowPassFilter2p_Update(&INS_AcceLPF2p[0],BMI088_Info.accel[IMU_ACCEL_INDEX_PITCH]);
    INS_Info.accel[IMU_ACCEL_INDEX_YAW]   = LowPassFilter2p_Update(&INS_AcceLPF2p[2],BMI088_Info.accel[IMU_ACCEL_INDEX_YAW]);
    INS_Info.accel[IMU_ACCEL_INDEX_ROLL]  = LowPassFilter2p_Update(&INS_AcceLPF2p[1],BMI088_Info.accel[IMU_ACCEL_INDEX_ROLL]);
		
    /* Update the INS gyro in radians */
		memcpy(INS_Info.gyro,BMI088_Info.gyro,sizeof(INS_Info.gyro));
		
		/* Update the QuaternionEKF */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.gyro,INS_Info.accel,0.001f);
		
    memcpy(INS_Info.angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.angle));

		/* Update the Euler angle in degrees. */
    INS_Info.pit_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.yaw_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]*RadiansToDegrees;
    INS_Info.rol_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*RadiansToDegrees;
		
		/* Update the yaw total angle */
		if(INS_Info.yaw_angle - INS_Info.last_yawangle < -180.f)
		{
			INS_Info.YawRoundCount++;
		}
		else if(INS_Info.yaw_angle - INS_Info.last_yawangle > 180.f)
		{
			INS_Info.YawRoundCount--;
		}
		INS_Info.last_yawangle = INS_Info.yaw_angle;
		
		INS_Info.yaw_tolangle = INS_Info.yaw_angle + INS_Info.YawRoundCount*360.f;
		
    /* Update the INS gyro in degrees */
    INS_Info.pit_gyro = INS_Info.gyro[IMU_GYRO_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.yaw_gyro = INS_Info.gyro[IMU_GYRO_INDEX_YAW]*RadiansToDegrees;
    INS_Info.rol_gyro = INS_Info.gyro[IMU_GYRO_INDEX_ROLL]*RadiansToDegrees;
		
		if(systick%5 == 0)
		{
			BMI088_Temp_Control(BMI088_Info.temperature);
		}

    //printf("%.2f\r\n",INS_Info.pit_angle);
    osDelayUntil(&systick,1);
  }
  /* USER CODE END INS_Task */
}
//------------------------------------------------------------------------------
/**
 * @brief Initializes the INS_Task.
 */
static void INSTask_Init(void)
{
  /* Initializes the Second order lowpass filter  */
  LowPassFilter2p_Init(&INS_AcceLPF2p[0],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AcceLPF2p[1],INS_LPF2p_Alpha);
  LowPassFilter2p_Init(&INS_AcceLPF2p[2],INS_LPF2p_Alpha);
	
  /* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID,PID_VELOCITY,TemCtrl_PID_Param);
	
  /* Initializes the Quaternion EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.f, 0.001f, 1000000.f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
}
//------------------------------------------------------------------------------
/**
  * @brief  Control the BMI088 temperature
  * @param  temp  measure of the BMI088 temperature
  * @retval none
  */
static void BMI088_Temp_Control(float temp)
{
	f_PID_Calculate(&TempCtrl_PID,40.f,temp);
	
	VAL_LIMIT(TempCtrl_PID.Output,0,2000);

	Heat_Power_Control((uint16_t)TempCtrl_PID.Output);
}
//------------------------------------------------------------------------------
const float *get_INS_angle_point(){
  return &INS_Info.angle[0];
}
const float *get_gyro_data_point(){
  return &INS_Info.gyro[0];
}
const float *get_accel_data_point(){
  return &INS_Info.accel[0];
}

