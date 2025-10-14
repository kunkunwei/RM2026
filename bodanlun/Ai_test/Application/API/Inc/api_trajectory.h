/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : api_trajectory.h
  * @brief          : solve trajectory
  * @author         : Yan Yuanbin
  * @date           : 2023/05/21
  * @version        : v1.0
  ******************************************************************************
  * @attention      : see https://github.com/chenjunnn/rm_auto_aim
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef API_TRAJECTORY_H
#define API_TRAJECTORY_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "minipc.h"

/* Exported defines -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the target armor posure.
 */
// 目标装甲位置姿态信息结构体
typedef struct
{
    float x;       /*!< x in the ros coordinate system */ // ROS坐标系下的x坐标
    float y;       /*!< y in the ros coordinate system */ // ROS坐标系下的y坐标
    float z;       /*!< z in the ros coordinate system */ // ROS坐标系下的z坐标
    float yaw;     /*!< target yaw angle */           // 目标偏航角
}TargetArmor_Posure;

/**
 * @brief typedef structure that contains the information for the solved trajectory.
 */
// 求解轨迹信息结构体
typedef struct
{
    float Camera_Muzzle_vertical;    /*!< the vertical distance of yaw axis to the muzzle(m) */ // 偏航轴到枪口的垂直距离（米）
    float Camera_Muzzle_horizontal;  /*!< the horizontal distance of yaw axis to the muzzle(m) */ // 偏航轴到枪口的水平距离（米）
    float FireSystem_BiasTime;       /*!< the bias time of system(s), contains the communication delay and trigger delay */ // 系统偏差时间（秒），包含通信延迟和触发延迟

    float bullet_speed;   /*!< referee bullet speed */ // 子弹初速度
    float bullet_time;    /*!< ballistic time */      // 弹道时间
    float current_pitch;  /*!< current pitch angle */ // 当前俯仰角
    float current_yaw;    /*!< current yaw angle */   // 当前偏航角

    float yaw_calc;       /*!< yaw angle in algorithm */ // 算法计算得到的偏航角
    float yawgyro_calc;   /*!< yaw gyro in algorithm */ // 算法计算得到的偏航陀螺角速度
    float r1;             /*!< Distance of target center to front and rear armor plates */ // 目标中心到前后装甲板距离
    float r2;             /*!< Distance of target center to armor plates in sides */ // 目标中心到侧面装甲板距离
    float dz;             /*!< unknown */               // 高度偏差未知量
    uint8_t armors_num;   /*!< the num of armor */       // 装甲板数量

    float armorlock_yaw;      /*!< gimbal target yaw angle,lock the armor */  // 云台目标偏航角，用于锁定装甲
    float armorlock_pitch;    /*!< gimbal target pitch angle,lock the armor  */ // 云台目标俯仰角，用于锁定装甲
    float centerlock_pitch;   /*!< gimbal target pitch angle,lock the center  */ // 云台目标俯仰角，用于锁定中心
    float centerlock_yaw;     /*!< gimbal target yaw angle,lock the center   */ // 云台目标偏航角，用于锁定中心

    float armor_distance;      // 装甲距离
    float center_distance;     // 中心距离

    TargetArmor_Posure target_posure[4];    /* target armor posure */ // 目标装甲阵列位置
}SolveTrajectory_Typedef;

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief  Update the solve trajectory 
 */
// 更新求解轨迹数据
extern void SolveTrajectory_Update(SolveTrajectory_Typedef *SolveTrajectory,float picth,float yaw,float target_yaw,float v_yaw,float r1,float r2,float dz,float bullet_speed,float armors_num);
/**
 * @brief  Transform the solve trajectory 
 */
// 转换并生成发送数据
extern void SolveTrajectory_Transform(MiniPC_SendPacket_Typedef *MiniPCTxData,MiniPC_ReceivePacket_Typedef *MiniPCRxData,SolveTrajectory_Typedef *SolveTrajectory);


#endif
