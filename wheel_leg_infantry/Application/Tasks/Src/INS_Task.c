/* USER CODE BEGIN Header */
#include <stdio.h>
/**
  ******************************************************************************
  * @file           : INS_Task.c
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "INS_Task.h"
#include "bsp_tim.h"
#include "bmi088.h"
#include "api_quaternion.h"
#include "bsp_dwt.h"
#include "lpf.h"
#include "pid.h"
#include "config.h"
#include "ist8310.h"
#include "mahony.h"
#include "main.h"

/**
 * 机体坐标系定义 (右手坐标系):
 *
 *      ↑ Y (Roll轴 - 横滚)
 *      |     ↑ Z (Yaw轴 - 偏航)
 *      |    /
 *      |   /
 *      |  /
 *      | /
 *      +------→ X (Pitch轴 - 俯仰)
 *
 * 传感器数据索引:
 * - 欧拉角:  [0]=Yaw(Z), [1]=Roll(Y), [2]=Pitch(X)
 * - 陀螺仪:  [0]=Pitch(X), [1]=Roll(Y), [2]=Yaw(Z)
 * - 加速度计: [0]=Pitch(X), [1]=Roll(Y), [2]=Yaw(Z)
 *
 * 旋转顺序: Z(Yaw) → Y(Roll) → X(Pitch)
 */
/**
  * @brief the structure that contains the information for the INS.
  */
INS_Info_Typedef INS_Info;
/* Private variables ---------------------------------------------------------*/

LowPassFilter1p_Info_TypeDef mag_lpf[3];

/**
  * @brief the array that contains the data of LPF2p coefficients.
  */
static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
/**
  * @brief the structure that contains the Information of accel LPF2p.
  */
LowPassFilter2p_Info_TypeDef INS_AcceLPF2p[3];  

/**
  * @brief the Initialize data of state transition matrix.
  */
static float QuaternionEKF_A_Data[36]={1, 0, 0, 0, 0, 0,
                                0, 1, 0, 0, 0, 0,
                                0, 0, 1, 0, 0, 0,
                                0, 0, 0, 1, 0, 0,
                                0, 0, 0, 0, 1, 0,
                                0, 0, 0, 0, 0, 1};
/**
  * @brief the Initialize data of posteriori covariance matrix.
  */
static float QuaternionEKF_P_Data[36]= {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                        0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                        0.1, 0.1, 0.1, 0.1, 0.1, 100};
// static float QuaternionEKF_P_Data[36] = {
//   // q0      q1      q2      q3      bias_x  bias_y
//   1.0f,    0.0f,   0.0f,   0.0f,   0.0f,   0.0f,    // q0
//   0.0f,    1.0f,   0.0f,   0.0f,   0.0f,   0.0f,    // q1
//   0.0f,    0.0f,   1.0f,   0.0f,   0.0f,   0.0f,    // q2
//   0.0f,    0.0f,   0.0f,   1.0f,   0.0f,   0.0f,    // q3
//   0.0f,    0.0f,   0.0f,   0.0f,   0.01f,  0.0f,    // bias_x (约5.7°/s的不确定性)
//   0.0f,    0.0f,   0.0f,   0.0f,   0.0f,   0.01f    // bias_y (约5.7°/s的不确定性)
// };
/**
  * @brief the Initialize data of Temperature Control PID.
  */
static float TemCtrl_PID_Param[PID_PARAMETER_NUM]={1600,20,0,0,0,2000};
/**
  * @brief the structure that contains the Information of Temperature Control PID.
  */
PID_Info_TypeDef TempCtrl_PID;
/**
  * @brief the structure that contains the Information of Quaternion.
  */
Quaternion_Info_Typedef Quaternion_Info;
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Initializes the INS_Task.
 */
static void INSTask_Init(void);
/**
  * @brief  Control the BMI088 temperature
  */
static void BMI088_Temp_Control(float temp);

void Euler_To_Quaternion(const float euler[3], float q[4]);
//------------------------------------------------------------------------------
/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINSTask thread.
  * @param  argument: Not used
  * @retval None
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
      /* Update the IST8310 measurement */
      // IST8310_Info_Update(&ist8310_Info);
      /*在新的场地可以解开这个注释，重新进行磁力计校准*/
  	// simple_mag_calibration_messure(&ist8310_Info);

    /*磁力计一阶低通滤波*/
      // ist8310_Info.calibrated_mag[0]=LowPassFilter1p_Update(&mag_lpf[0],ist8310_Info.calibrated_mag[0]);
      // ist8310_Info.calibrated_mag[1]=LowPassFilter1p_Update(&mag_lpf[1],ist8310_Info.calibrated_mag[1]);
      // ist8310_Info.calibrated_mag[2]=LowPassFilter1p_Update(&mag_lpf[2],ist8310_Info.calibrated_mag[2]);
    /* accel measurement LPF2p */
    INS_Info.accel[IMU_ACCEL_INDEX_PITCH] = LowPassFilter2p_Update(&INS_AcceLPF2p[0],BMI088_Info.accel[IMU_ACCEL_INDEX_PITCH]);
    INS_Info.accel[IMU_ACCEL_INDEX_YAW]   = LowPassFilter2p_Update(&INS_AcceLPF2p[2],BMI088_Info.accel[IMU_ACCEL_INDEX_YAW]);
    INS_Info.accel[IMU_ACCEL_INDEX_ROLL]  = LowPassFilter2p_Update(&INS_AcceLPF2p[1],BMI088_Info.accel[IMU_ACCEL_INDEX_ROLL]);

    /* Update the INS gyro in radians */
		memcpy(INS_Info.gyro,BMI088_Info.gyro,sizeof(INS_Info.gyro));

		/* Update the QuaternionEKF */
    QuaternionEKF_Update(&Quaternion_Info,INS_Info.gyro,INS_Info.accel,0.001f);

      // YawFusion_UpdateMag(&yaw_fusion, ist8310_Info.calibrated_mag);
      // YawFusion_Update(&yaw_fusion, Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW], DWT_GetTimeline_ms());
    memcpy(INS_Info.angle,Quaternion_Info.EulerAngle,sizeof(INS_Info.angle));

		/* Update the Euler angle in degrees. */
    INS_Info.pit_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_PITCH]*RadiansToDegrees;
    INS_Info.yaw_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_YAW]*RadiansToDegrees;
    INS_Info.rol_angle = Quaternion_Info.EulerAngle[IMU_ANGLE_INDEX_ROLL]*RadiansToDegrees;
    // YawFusion_GetCorrectedYaw(&INS_Info.yaw_angle);
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

    // LowPassFilter1p_Init(&mag_lpf[0],0.05f,0.001f);
    // LowPassFilter1p_Init(&mag_lpf[1],0.05f,0.001f);
    // LowPassFilter1p_Init(&mag_lpf[2],0.05f,0.001f);
  /* Initializes the Temperature Control PID  */
	PID_Init(&TempCtrl_PID,PID_VELOCITY,TemCtrl_PID_Param);

  /* Initializes the Quaternion EKF */
	QuaternionEKF_Init(&Quaternion_Info,10.0f, 0.001f, 1000000.0f,QuaternionEKF_A_Data,QuaternionEKF_P_Data);
    // YawFusion_Init(&yaw_fusion);
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
const Quaternion_Info_Typedef *get_quaternion_info_point(){
  return &Quaternion_Info;
}
