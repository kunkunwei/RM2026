/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_quaternion.h
  * @brief          : quaternion fusion api
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_QUATERNION_H
#define API_QUATERNION_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "kalman.h"


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the Information of Quaternion.
 */
// 四元数融合信息结构体
typedef struct
{
  bool init; /*!< Initialized flag */ // 是否已初始化标志

  float quat[4];       /*!< the data of quaternion */ // 四元数数据
  float deviate[3];    /*!< the deviate of Gyro */     // 陀螺仪偏差

  float Q1,Q2,R;       /*!< the data of process/measurement noise covariance matrix */ // 过程/测量噪声协方差
  float *QuaternionEKF_A_Data;  /*!< pointer to the data of state transition matrix */ // 状态转移矩阵数据指针
  float *QuaternionEKF_P_Data;  /*!< pointer to the data of posteriori covariance matrix */ // 后验协方差矩阵数据指针
  KalmanFilter_Info_TypeDef QuaternionEKF;  /*!< Extended Kalman Filter */ // 扩展卡尔曼滤波器实例

  float accel[3];      /*!< the data of accel measure */ // 加速度计测量数据
  float gyro[3];       /*!< the data of gyro measure */  // 陀螺仪测量数据
  float accelInvNorm;       /*!< the inverse of accel norm */ // 加速度计模反
  float gyroInvNorm;        /*!< the inverse of gyro norm */  // 陀螺仪模反
  float halfgyrodt[3];      /*!< half gyro dt */           // 半个陀螺积分量
  float EulerAngle[3];      /*!< Euler angles in radians: */ // 欧拉角，弧度
}Quaternion_Info_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief Initializes the Quaternion EKF according to the specified parameters in the Quaternion_Info_Typedef.
  */
// 初始化四元数扩展卡尔曼滤波器
extern void QuaternionEKF_Init(Quaternion_Info_Typedef *quat,float process_noise1,float process_noise2,float measure_noise,float *QuaternionEKF_A_Data,float *QuaternionEKF_P_Data);
/**
  * @brief  Update the Extended Kalman Filter
  */
// 更新扩展卡尔曼滤波器，融合新测量
extern void QuaternionEKF_Update(Quaternion_Info_Typedef *quat,float gyro[3],float accel[3],float dt);
// extern void Quaternion_To_Euler(const float q[4], float euler[3]);
#endif //API_QUATERNION_H
