/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : 电机设备接口函数头文件
  * @details        : 支持多种类型电机的统一接口，包括：
  *                   - DJI无刷电机（GM6020、M3508、M2006）
  *                   - RMD减速电机（L9025）
  *                   - 达妙电机（关节电机）
  *                   提供电机数据更新、状态监控、错误处理等功能
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 需要配合CAN总线通信使用
  ******************************************************************************
  */
/* USER CODE END Header */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MOTOR_H
#define DEVICE_MOTOR_H


/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "stm32f4xx.h"
#include "pid.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief RMD电机使用枚举
 * @details 定义RMD减速电机在系统中的具体用途和数量
 */
typedef enum{
	Left_Wheel,             // 左轮电机
	Right_Wheel,            // 右轮电机
  RMD_MOTOR_USAGE_NUM,    // RMD电机总数量
}RMD_MOTOR_USAGE_e;

/**
 * @brief DJI电机使用枚举
 * @details 定义DJI无刷电机在系统中的具体用途和数量
 */
typedef enum{
	Left_Momentum,          // 左侧动量轮电机
	Right_Momentum,         // 右侧动量轮电机
	Yaw,                    // YAW轴云台电机
  DJI_MOTOR_USAGE_NUM,    // DJI电机总数量
}DJI_MOTOR_USAGE_e;

/**
 * @brief 达妙电机使用枚举
 * @details 定义达妙关节电机在机器人腿部关节中的具体位置
 */
typedef enum{
	Left_Anterior_Joint,     // 左前关节电机
  Left_Posterior_Joint,    // 左后关节电机
	Right_Anterior_Joint,    // 右前关节电机
  Right_Posterior_Joint,   // 右后关节电机
	Damiao_MOTOR_USAGE_NUM   // 达妙电机总数量
}Damiao_MOTOR_USAGE_e;
/**
 * @brief DJI电机CAN帧标识符枚举
 * @details 定义DJI电机CAN通信的标准帧ID，用于电机控制和数据接收
 */
typedef enum
{
  DJI_TxFrame_HIGH = 0x1ffU,    // 高ID发送帧（0x205-0x208电机控制）
  DJI_TxFrame_LOW = 0x200U,     // 低ID发送帧（0x201-0x204电机控制）
  DJI_RxFrame_MIDDLE = 0x204U,  // 中间接收帧基准ID
  DJI_MotorFrameId_NUM,         // 帧ID枚举数量
}DJI_MotorFrameId_e;


/**
 * @brief 电机设备错误状态枚举
 * @details 定义电机运行过程中可能出现的各种错误状态
 */
typedef enum
{
  MOTOR_ERROR_NONE = 0x00U,         /*!< 无错误，正常状态 */
  MOTOR_CAN_OFFLINE = 0x01U,        /*!< CAN通信离线或失败 */
  MOTOR_OVER_TEMPERATURE = 0x02U,   /*!< 电机过温异常 */
}Motor_Status_e;

/**
 * @brief RMD电机类型枚举
 * @details 定义支持的RMD减速电机型号
 */
typedef enum{
	  RMD_L9025,              // RMD L9025型减速电机
    RMD_MOTOR_TYPE_NUM,     // RMD电机类型总数
}RMD_Motor_Type_e;

/**
 * @brief DJI电机类型枚举
 * @details 定义支持的DJI无刷电机型号
 */
typedef enum{
    DJI_GM6020,             // GM6020云台电机（无减速器）
    DJI_M3508,              // M3508底盘电机（3591/187减速比）
    DJI_M2006,              // M2006小功率电机（36:1减速比）
    DJI_MOTOR_TYPE_NUM,     // DJI电机类型总数
}DJI_Motor_Type_e;

/**
 * @brief 电机错误处理器结构体
 * @details 包含电机错误状态监测和计数信息
 */
typedef struct 
{
  uint16_t ErrorCount;        /*!< 错误状态判断计数器 */
  Motor_Status_e Status;      /*!< 当前错误状态 */
}Motor_ErrorrHandler_Typedef;

/**
 * @brief 电机CAN通信帧信息结构体
 * @details 包含电机CAN通信所需的发送和接收帧ID以及帧索引
 */
typedef struct
{
  uint32_t TxStdId;     /*!< CAN发送标准帧ID */
  uint32_t RxStdId;     /*!< CAN接收标准帧ID */
  uint8_t FrameIndex;   /*!< 电机在发送帧中的索引位置 */
}Motor_CANFrameInfo_typedef;

/**
 * @brief 电机设备通用信息结构体
 * @details 包含电机运行的基本状态信息
 */
typedef struct 
{
  bool Initlized;           /*!< 初始化标志位 */

  int16_t  current;         /*!< 电机输出电流（mA） */
  int16_t  velocity;        /*!< 电机转速（RPM） */
  int16_t  encoder;         /*!< 电机编码器角度值 */
  int16_t  last_encoder;    /*!< 上一次编码器角度值 */
  float    angle;           /*!< 电机累积转角（度） */
  uint8_t  temperature;     /*!< 电机温度（°C） */
}Motor_GeneralInfo_Typedef;
/**
 * @brief RMD电机通用信息结构体
 * @details 包含RMD减速电机的特定状态信息
 */
typedef struct 
{
  bool Initlized;           /*!< 初始化标志位 */

  int16_t  Current;         /*!< 电机输出电流（A*100） */
  int16_t  Velocity;        /*!< 电机转速（度/秒*100） */
	float    Rad_Velocity;    /*!< 电机转速（弧度/秒） */
  float    Angle;           /*!< 电机角度（度） */
  uint8_t  Temperature;     /*!< 电机温度（°C） */
}RMD_GeneralInfo_Typedef;
/**
 * @brief 达妙电机通用信息结构体
 * @details 包含达妙关节电机的详细状态和反馈信息
 */
typedef struct 
{
  bool Initlized;               /*!< 初始化标志位 */
  int16_t  State; 	            /*!< 电机错误状态信息 */
  uint16_t  P_int;              /*!< 位置反馈原始数据（整型） */
	uint16_t  V_int;              /*!< 速度反馈原始数据（整型） */
	uint16_t  T_int;              /*!< 扭矩反馈原始数据（整型） */
	float  Position;              /*!< 电机位置（弧度） */
  float  Velocity;              /*!< 电机速度（弧度/秒） */
  float  Torque;                /*!< 电机扭矩（N·m） */
  float  Temperature_MOS;       /*!< MOS管温度（°C） */
	float  Temperature_Rotor;     /*!< 转子温度（°C） */
  float  Angle;	                /*!< 电机角度（度） */
}Damiao_GeneralInfo_Typedef;
/**
 * @brief DJI电机设备信息结构体
 * @details 包含DJI电机的完整配置和状态信息
 */
typedef struct
{
	DJI_Motor_Type_e Type;                      /*!< 电机型号类型 */
  Motor_CANFrameInfo_typedef CANFrame;        /*!< CAN通信帧信息 */
	Motor_GeneralInfo_Typedef Data;             /*!< 电机设备通用数据 */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< 电机错误处理信息 */
}DJI_Motor_Info_Typedef;

/**
 * @brief 达妙电机设备信息结构体
 * @details 包含达妙关节电机的完整配置和状态信息
 */
typedef struct
{
	uint8_t ID;                                 /*!< 电机ID号 */
  Motor_CANFrameInfo_typedef CANFrame;        /*!< CAN通信帧信息 */
	Damiao_GeneralInfo_Typedef Data;            /*!< 达妙电机专用数据 */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< 电机错误处理信息 */
}Damiao_Motor_Info_Typedef;

/**
 * @brief 达妙电机控制信息结构体
 * @details 包含达妙电机的控制参数和目标值
 */
typedef struct
{
  float Position;       // 目标位置（弧度）
	float Velocity;       // 目标速度（弧度/秒）
	float KP;             // 位置比例增益
	float KD;             // 速度比例增益
	float Torque;         // 前馈扭矩（N·m）
	float Angle;          // 目标角度（度）
}Damiao_Motor_Contorl_Info_Typedef;
/**
 * @brief RMD L9025电机设备信息结构体
 * @details 包含RMD减速电机的完整配置和状态信息
 */
typedef struct
{
	uint8_t order;                              /*!< 电机反馈指令序号 */
	RMD_Motor_Type_e Type;                      /*!< 电机型号类型 */
  Motor_CANFrameInfo_typedef CANFrame;        /*!< CAN通信帧信息 */
	RMD_GeneralInfo_Typedef Data;               /*!< RMD电机专用数据 */
	Motor_ErrorrHandler_Typedef ERRORHandler;   /*!< 电机错误处理信息 */
}RMD_L9025_Info_Typedef;


/* 全局变量声明 */
extern RMD_L9025_Info_Typedef RMD_Motor[RMD_MOTOR_USAGE_NUM];           // RMD电机信息数组
extern DJI_Motor_Info_Typedef DJI_Motor[DJI_MOTOR_USAGE_NUM];           // DJI电机信息数组
extern Damiao_Motor_Info_Typedef Damiao_Motor[Damiao_MOTOR_USAGE_NUM];   // 达妙电机信息数组
extern Damiao_Motor_Contorl_Info_Typedef  Damiao_Motor_Contorl_Info[4];  // 达妙电机控制信息数组

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief 更新DJI电机信息
  * @param StdId CAN标准帧ID指针
  * @param rxBuf CAN接收数据缓冲区指针
  * @param DJI_Motor DJI电机信息结构体指针
  * @details 解析CAN接收数据，更新电机状态信息
  */
extern void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor);

/**
  * @brief 更新RMD电机信息
  * @param StdId CAN标准帧ID指针
  * @param rxBuf CAN接收数据缓冲区指针
  * @param RMD_Motor RMD电机信息结构体指针
  * @details 解析CAN接收数据，更新RMD电机状态信息
  */
extern void RMD_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,RMD_L9025_Info_Typedef *RMD_Motor);

/**
  * @brief 更新达妙电机信息
  * @param rxBuf CAN接收数据缓冲区指针
  * @param Damiao_Motor 达妙电机信息结构体指针
  * @details 解析CAN接收数据，更新达妙关节电机状态信息
  */
extern void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor);

/**
  * @brief 使能达妙电机
  * @param ID 电机ID号
  * @details 发送使能指令，激活达妙电机
  */
extern void  Damiao_Motor_Enable(uint8_t ID);

/**
  * @brief 达妙电机二次使能
  * @param ID 电机ID号
  * @details 发送二次使能指令
  */
extern void Damiao_Motor_2Enable(uint8_t ID);

/**
  * @brief 失能达妙电机
  * @param ID 电机ID号
  * @details 发送失能指令，停止达妙电机
  */
extern void  Damiao_Motor_DisEnable(uint8_t ID);

#endif //DEVICE_MOTOR_H
