/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : 电机接口函数
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待测试
  ******************************************************************************
  */
/* USER CODE END Header */


/* 防止递归包含 --------------------------------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* 头文件包含 ----------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx.h"
#include "pid.h"

/* 导出类型 ------------------------------------------------------------------*/


/**
 * @brief 枚举，包含DJI电机的用途
 */
typedef enum{
	Chassis,      // 底盘电机
	Pitch,     // 云台电机
	Yaw,                // 云台电机
    Shoot,              // 摩擦轮电机
  DJI_MOTOR_USAGE_NUM,// 电机用途数量
}DJI_MOTOR_USAGE_e;

/**
 * @brief 枚举，包含DJI电机的帧标识符
 */
typedef enum
{
  DJI_TxFrame_HIGH = 0x1ffU,      // 高帧ID
  DJI_TxFrame_LOW = 0x200U,       // 低帧ID
  DJI_RxFrame_MIDDLE = 0x204U,    // 中间帧ID
  DJI_MotorFrameId_NUM,           // 帧ID数量
}DJI_MotorFrameId_e;


/**
 * @brief 枚举，包含电机设备的错误状态
 */
typedef enum
{
  MOTOR_ERROR_NONE = 0x00U,           /*!< 无错误 */
  MOTOR_CAN_OFFLINE = 0x01U,          /*!< CAN通信失败 */
  MOTOR_OVER_TEMPERATURE = 0x02U,     /*!< 电机温度异常 */
}Motor_Status_e;

/**
 * @brief 枚举，包含DJI电机类型
 */
typedef enum{
    DJI_GM6020,           // GM6020型号
    DJI_M3508,            // M3508型号
    DJI_M2006,            // M2006型号
    DJI_MOTOR_TYPE_NUM,   // DJI电机类型数量
}DJI_Motor_Type_e;

/**
 * @brief 电机错误处理结构体
 */
typedef struct 
{
  uint16_t ErrorCount;    /*!< 错误状态计数 */
  Motor_Status_e Status;  /*!< 错误状态 */
}Motor_ErrorrHandler_Typedef;

/**
 * @brief 电机CAN通信帧信息结构体
 */
typedef struct
{
  uint32_t TxStdId;   /*!< CAN发送ID */
  uint32_t RxStdId;   /*!< CAN接收ID */
  uint8_t FrameIndex; /* 电机发送帧索引 */
}Motor_CANFrameInfo_typedef;

/**
 * @brief 通用电机信息结构体
 */
typedef struct 
{
  bool Initialized;           /*!< 初始化标志 */
  int16_t  current;         /*!< 电机电流 */
  int16_t  velocity;        /*!< 电机转速 */
  int16_t  encoder;         /*!< 电机编码器角度 */
  int16_t  last_encoder;    /*!< 上一次编码器角度 */
  float    angle;           /*!< 电机角度（度） */
  uint8_t  temperature;     /*!< 电机温度 */
}Motor_GeneralInfo_Typedef;


/**
 * @brief 达妙电机信息结构体
 */
typedef struct 
{
  bool Initlized;           /*!< 初始化标志 */
  int16_t  State;           /*!< 电机错误信息 */
  uint16_t  P_int;          /*!< 位置原始值 */
	uint16_t  V_int;          /*!< 速度原始值 */
	uint16_t  T_int;          /*!< 转矩原始值 */
	float  Position;          /*!< 电机位置（弧度） */
  float  Velocity;          /*!< 电机速度（弧度/秒） */
  float  Torque;            /*!< 电机转矩 */
  float  Temperature_MOS;   /*!< MOS温度 */
	float  Temperature_Rotor; /*!< 转子温度 */
  float  Angle;	            /*!< 电机角度（度） */
}Damiao_GeneralInfo_Typedef;

/**
 * @brief DJI电机设备信息结构体
 */
typedef struct
{
	DJI_Motor_Type_e Type;                   /*!< 电机类型 */
  Motor_CANFrameInfo_typedef CANFrame;      /*!< CAN通信信息 */
	Motor_GeneralInfo_Typedef Data;           /*!< 电机数据 */
	Motor_ErrorrHandler_Typedef ERRORHandler; /*!< 错误处理信息 */
}DJI_Motor_Info_Typedef;

/**
 * @brief 达妙电机设备信息结构体
 */
typedef struct
{
	uint8_t ID;                              /*!< 电机ID */
  Motor_CANFrameInfo_typedef CANFrame;      /*!< CAN通信信息 */
	Damiao_GeneralInfo_Typedef Data;          /*!< 电机数据 */
	Motor_ErrorrHandler_Typedef ERRORHandler; /*!< 错误处理信息 */
}Damiao_Motor_Info_Typedef;

/**
 * @brief 达妙电机控制信息结构体
 */
typedef struct
{
  float Position;   // 目标位置
	float Velocity;   // 目标速度
	float KP;         // 位置环增益
	float KD;         // 速度环增益
	float Torque;     // 目标转矩
	float Angle;      // 目标角度
}Damiao_Motor_Contorl_Info_Typedef;


/* 外部变量声明 --------------------------------------------------------------*/
extern DJI_Motor_Info_Typedef DJI_Motor[DJI_MOTOR_USAGE_NUM];

extern Damiao_Motor_Info_Typedef Damiao_Motor[Damiao_MOTOR_USAGE_NUM];

extern Damiao_Motor_Contorl_Info_Typedef  Damiao_Motor_Contorl_Info[4];

/* 导出函数声明 --------------------------------------------------------------*/
/**
  * @brief  更新DJI电机信息
  */
extern void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor);

/**
  * @brief  更新达妙电机信息
  */
extern void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor);

/**
  * @brief  使能大妙电机
  */
extern void  Damiao_Motor_Enable(uint8_t ID);
/**
  * @brief  使能大妙电机（备用）
  */
extern void Damiao_Motor_2Enable(uint8_t ID);
/**
  * @brief  关闭大妙电机使能
  */
extern void  Damiao_Motor_DisEnable(uint8_t ID);

#endif //DEVICE_MOTOR_H
