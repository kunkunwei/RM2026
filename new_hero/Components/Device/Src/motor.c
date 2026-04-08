/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : 电机设备接口函数实现文件
  * @details        : 实现多种类型电机的统一接口，包括：
  *                   - DJI无刷电机数据解析和控制
  *                   - RMD减速电机数据解析和控制
  *                   - 达妙关节电机数据解析和控制
  *                   - 编码器角度转换和累积计算
  *                   - 电机状态监测和错误处理
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 需要配合CAN总线通信和相应的BSP驱动使用
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include "bsp_can.h"

/**
 * @brief RMD电机信息结构体数组
 * @details 配置左右轮RMD减速电机的基本信息和CAN通信参数
 */
RMD_L9025_Info_Typedef RMD_Motor[RMD_MOTOR_USAGE_NUM]=
{
	[Left_Wheel]=
	{
		.Type = RMD_L9025,          // 电机型号：RMD L9025
		.CANFrame.RxStdId = 0x242,  // 左轮电机CAN接收ID
	},
	[Right_Wheel]=
	{
		.Type = RMD_L9025,          // 电机型号：RMD L9025
		.CANFrame.RxStdId = 0x241,  // 右轮电机CAN接收ID
	},
};

/**
 * @brief DJI电机信息结构体数组
 * @details 配置各轴DJI电机的基本信息和CAN通信参数
 */
DJI_Motor_Info_Typedef DJI_Motor[DJI_MOTOR_USAGE_NUM]=
{
	[Left_Momentum]=
	{
		.Type = DJI_GM6020,         // 电机型号：DJI GM6020
		.CANFrame.RxStdId = 0x205,  // 左动量轮电机CAN接收ID
	},
	[Right_Momentum]=
	{
		.Type = DJI_GM6020,         // 电机型号：DJI GM6020
		.CANFrame.RxStdId = 0x206,  // 右动量轮电机CAN接收ID
	},
	[Yaw]=
	{
		.Type = DJI_GM6020,         // 电机型号：DJI GM6020
		.CANFrame.RxStdId = 0x205,  // Yaw轴电机CAN接收ID
	},
};
/**
 * @brief 达妙电机信息结构体数组
 * @details 配置四个关节电机的CAN通信ID参数
 */
Damiao_Motor_Info_Typedef Damiao_Motor[Damiao_MOTOR_USAGE_NUM]=
{
    [Left_Anterior_Joint]=
	  {
		  .CANFrame.TxStdId = 0x03,  // 左前关节电机CAN发送ID
			.CANFrame.RxStdId = 0x03   // 左前关节电机CAN接收ID
		},
    [Left_Posterior_Joint]=
		{
		   .CANFrame.TxStdId = 0x04,   // 左后关节电机CAN发送ID
			 .CANFrame.RxStdId = 0x04,   // 左后关节电机CAN接收ID
		},
		[Right_Anterior_Joint]=
		{
		   .CANFrame.TxStdId = 0x01,   // 右前关节电机CAN发送ID
			 .CANFrame.RxStdId = 0x01,   // 右前关节电机CAN接收ID
		},
		[Right_Posterior_Joint]=
		{
			 .CANFrame.RxStdId = 0x02,   // 右后关节电机CAN接收ID
		   .CANFrame.TxStdId = 0x02,   // 右后关节电机CAN发送ID
		},
};
/**
 * @brief 达妙电机控制信息结构体数组
 * @details 初始化四个关节电机的控制参数，包括位置、速度、刚度等
 */
Damiao_Motor_Contorl_Info_Typedef Damiao_Motor_Contorl_Info[4]={
	[Left_Anterior_Joint]={
   .Position = 0,      // 目标位置：0 rad
	 .Velocity = 0,      // 目标速度：0 rad/s
	 .KP = 0,            // 位置刚度增益
	 .KD = 1,            // 速度阻尼增益
	 .Torque = 0,        // 前馈扭矩：0 N·m
	 .Angle = 0,         // 当前角度：0 rad
	},
	[Left_Posterior_Joint]={
   .Position = 0,      // 目标位置：0 rad
	 .Velocity = 0,      // 目标速度：0 rad/s
	 .KP = 0,            // 位置刚度增益
	 .KD = 1,            // 速度阻尼增益
	 .Torque = 0,        // 前馈扭矩：0 N·m
	 .Angle = 0,         // 当前角度：0 rad
	},
		[Right_Anterior_Joint]={
   .Position = 0,      // 目标位置：0 rad
	 .Velocity = 0,      // 目标速度：0 rad/s
	 .KP = 0,            // 位置刚度增益
	 .KD = 1,            // 速度阻尼增益
	 .Torque = 0,        // 前馈扭矩：0 N·m
	 .Angle = 0,         // 当前角度：0 rad
	},
		[Right_Posterior_Joint]={
   .Position = 0,      // 目标位置：0 rad
	 .Velocity = 0,      // 目标速度：0 rad/s
	 .KP = 0,            // 位置刚度增益
	 .KD = 1,            // 速度阻尼增益
	 .Torque = 0,        // 前馈扭矩：0 N·m
	 .Angle = 0,         // 当前角度：0 rad
	},
};

/**
 * @brief 达妙电机使能帧信息数组
 */
uint8_t DamiaoMotorAble_FramInfo[8];
uint8_t DamiaoMotor2Able_FramInfo[8];

//};
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  编码器值(0-8192)转换为累计角度值(3.4E38)
 * @param  Motor_GeneralInfo_Typedef* 电机通用信息结构体指针
 * @param  float 角度倍数
 * @param  uint16_t 编码器原始值
 * @return float 累计角度值
 */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *,float ,uint16_t );

/**
 * @brief  编码器值(0-8192)转换为角度值(-180~180度)
 * @param  Motor_GeneralInfo_Typedef* 电机通用信息结构体指针
 * @param  float 角度倍数
 * @param  uint16_t 编码器原始值
 * @return float 角度值
 */
float encoder_to_angle(Motor_GeneralInfo_Typedef *,float ,uint16_t );

/**
 * @brief  无符号整型转换为浮点型
 * @param  X_int 输入整型数值
 * @param  X_min 最小浮点值
 * @param  X_max 最大浮点值
 * @param  Bits 数据位数
 * @return float 转换后的浮点值
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

/**
 * @brief  DJI电机错误处理函数
 * @param  DJI_Motor_Info_Typedef* DJI电机信息结构体指针
 */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *);

/**
 * @brief  达妙电机错误处理函数
 * @param  Damiao_Motor_Info_Typedef* 达妙电机信息结构体指针
 */
static void Damiao_Motor_ErrorHandler(Damiao_Motor_Info_Typedef *Damiao_Motor);
/**
 * @brief  更新DJI电机信息
 * @param  StdId 标准CAN ID指针
 * @param  rxBuf CAN接收数据缓冲区指针
 * @param  DJI_Motor DJI电机信息结构体指针，包含电机的所有信息
 * @retval None
 */
void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* 检查CAN标准ID是否匹配 */
	if(*StdId != DJI_Motor->CANFrame.RxStdId) return;
	
	/* 解析电机反馈数据 */
	DJI_Motor->Data.temperature = rxBuf[6];  // 电机温度
	DJI_Motor->Data.encoder  = ((int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1]);  // 编码器值
	DJI_Motor->Data.velocity = ((int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3]);  // 转速
	DJI_Motor->Data.current  = ((int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5]);  // 电流
	/* 电机错误状态判断 */
	DJI_Motor_ErrorHandler(DJI_Motor);

  /* 更新发送帧ID和索引 */
  if(DJI_Motor->Data.Initlized != true)
  {
    if(DJI_Motor->CANFrame.RxStdId > DJI_RxFrame_MIDDLE)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_HIGH;  // 设置高位发送ID
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_RxFrame_MIDDLE - 0x01U);
    }
    else if(DJI_Motor->CANFrame.RxStdId > DJI_TxFrame_LOW)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_LOW;  // 设置低位发送ID
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_TxFrame_LOW - 0x01U);
    }
  }
	
	/* 编码器值转换为累计角度值 */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,1.f,8192);  // GM6020直驱
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,3591.f/187.f,8192);  // M3508减速比19.2:1
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,36.f,8192);  // M2006减速比36:1
		break;
		
		default:break;
	}
}
//------------------------------------------------------------------------------
/**
 * @brief  更新达妙电机信息
 * @param  rxBuf CAN接收数据缓冲区指针
 * @param  Damiao_Motor 达妙电机信息结构体指针，包含电机的所有信息
 * @retval None
 */
 
void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor)
{
	/* 判断电机ID */
  	Damiao_Motor->ID = rxBuf[0]&0x0F;  // 提取电机ID（低4位）

	 if(Damiao_Motor->ID != Damiao_Motor->CANFrame.RxStdId) return;

	  /* 解析达妙电机反馈数据 */
	  Damiao_Motor->Data.State = rxBuf[0]>>4;  // 电机运行状态（高4位）
		Damiao_Motor->Data.P_int = ((uint16_t)(rxBuf[1]) <<8) | ((uint16_t)(rxBuf[2]));       // 位置整型数据
		Damiao_Motor->Data.V_int = ((uint16_t)(rxBuf[3]) <<4) | ((uint16_t)(rxBuf[4])>>4);   // 速度整型数据  
		Damiao_Motor->Data.T_int = ((uint16_t)(rxBuf[4]&0xF) <<8) | ((uint16_t)(rxBuf[5]));  // 扭矩整型数据

		/* 整型数据转换为浮点型物理量 */
		Damiao_Motor->Data.Torque=  uint_to_float(Damiao_Motor->Data.T_int,-45,45,12);         // 扭矩: -45~45 N·m
		Damiao_Motor->Data.Position=uint_to_float(Damiao_Motor->Data.P_int,-3.141593,3.141593,16);  // 位置: -π~π rad
		Damiao_Motor->Data.Velocity=uint_to_float(Damiao_Motor->Data.V_int,-50,50,12);         // 速度: -50~50 rad/s

   	//if(fabs(Damiao_Motor->Data.Velocity)<0.015) Damiao_Motor->Data.Velocity = 0;
		/* 温度数据解析 */
		Damiao_Motor->Data.Temperature_MOS   = (float)(rxBuf[6]);    // MOS管温度
		Damiao_Motor->Data.Temperature_Rotor = (float)(rxBuf[7]);    // 转子温度
		Damiao_Motor->Data.Angle =  Damiao_Motor->Data.Position*Rad_to_angle;  // 弧度转角度
    
  /* 根据电机ID进行位置校准和坐标转换 */
  if(Damiao_Motor->CANFrame.RxStdId == 0x01){
    Damiao_Motor->Data.Position -= 0.689898f;                       // 右前关节零点校准
    Damiao_Motor->Data.Position = 3.141593/2 - Damiao_Motor->Data.Position;  // 坐标系转换
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x02){
    Damiao_Motor->Data.Position += 0.504883f;                       // 右后关节零点校准
    Damiao_Motor->Data.Position = 3.141593/2 - Damiao_Motor->Data.Position;  // 坐标系转换
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x03){
    Damiao_Motor->Data.Position += 1.144236f;                       // 左前关节零点校准
    Damiao_Motor->Data.Position += 3.141593/2;                      // 坐标系转换
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x04){  // 修正ID判断逻辑错误
    Damiao_Motor->Data.Position += 0.473602f;                       // 左后关节零点校准
    Damiao_Motor->Data.Position += 3.141593/2;                      // 坐标系转换
  }

	/* 电机状态错误检测 */
	if(Damiao_Motor->Data.State!=0){
   Damiao_Motor_ErrorHandler(Damiao_Motor);  // 调用错误处理函数
	}
}

//------------------------------------------------------------------------------

/**
 * @brief  编码器值(0-8192)转换为累计角度值(3.4E38)
 * @param  *Info        电机通用信息结构体指针，包含指定电机的信息
 * @param  torque_ratio 指定电机的扭矩比例系数
 * @param  MAXencoder   指定电机的最大编码器值
 * @retval anglesum     累计角度值
 */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(Info == NULL) return 0;
  
  /* 判断电机是否已初始化 */
  if(Info->Initlized != true)
  {
    /* 更新上次编码器值 */
    Info->last_encoder = Info->encoder;

    /* 重置角度值 */
    Info->angle = 0;

    /* 设置初始化标志 */
    Info->Initlized = true;
  }
  
  /* 计算可能的最小编码器误差 */
  if(Info->encoder < Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder + MAXencoder;  // 正向溢出处理
  }
  else if(Info->encoder > Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder - MAXencoder;  // 负向溢出处理
  }
  res2 = Info->encoder - Info->last_encoder;  // 直接差值
  
  /* 更新上次编码器值 */
  Info->last_encoder = Info->encoder;
  
  /* 编码器数据转换为总角度 */
	if(fabsf(res1) > fabsf(res2))
	{
		Info->angle += (float)res2/(MAXencoder*torque_ratio)*360.f;  // 使用直接差值
	}
	else
	{
		Info->angle += (float)res1/(MAXencoder*torque_ratio)*360.f;  // 使用溢出处理后的差值
	}
  
  return Info->angle;
}
//------------------------------------------------------------------------------

/**
 * @brief  浮点数循环约束函数
 * @param  Input    指定的变量值
 * @param  minValue 指定变量的最小值
 * @param  maxValue 指定变量的最大值
 * @retval variables 约束后的变量值
 */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;  // 参数错误，直接返回输入值
  }
  
  float len = maxValue - minValue;  // 计算区间长度

  if (Input > maxValue)
  {
      do{
          Input -= len;  // 循环减去区间长度
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;  // 循环加上区间长度
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
 * @brief  编码器值(0-8192)转换为角度值(-180~180度)
 * @param  *Info        电机通用信息结构体指针，包含指定电机的信息
 * @param  torque_ratio 指定电机的扭矩比例系数
 * @param  MAXencoder   指定电机的最大编码器值
 * @retval angle        角度值(-180~180度)
 */
float encoder_to_angle(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{	
  float encoder_err = 0.f;
  
  /* 检查电机初始化状态 */
  if(Info->Initlized != true)
  {
    /* 更新上次编码器值 */
    Info->last_encoder = Info->encoder;

    /* 重置角度值 */
    Info->angle = 0;

    /* 配置初始化标志 */
    Info->Initlized = true;
  }
  
  encoder_err = Info->encoder - Info->last_encoder;
  
  /* 编码器从0跳转到MAXencoder(反向溢出) */		
  if(encoder_err > MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  /* 编码器从MAXencoder跳转到0(正向溢出) */		
  else if(encoder_err < -MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  else
  {
    Info->angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;  // 正常增量
  }
  
  /* 更新上次编码器值 */
  Info->last_encoder = Info->encoder;
  
  /* 角度循环约束 */
  Info->angle = f_loop_constrain(Info->angle,-180.f,180.f);  // 限制在-180~180度范围内

  return Info->angle;
}
//------------------------------------------------------------------------------

/** 
 * @brief  判断DJI电机状态
 * @param  *DJI_Motor DJI电机信息结构体指针，包含指定电机的配置信息
 * @retval None
 */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* 判断DJI电机温度 */
	if(DJI_Motor->Data.temperature > 80)
	{
    DJI_Motor->ERRORHandler.ErrorCount++;  // 错误计数增加

    if(DJI_Motor->ERRORHandler.ErrorCount > 200)
    {
      DJI_Motor->ERRORHandler.Status = MOTOR_OVER_TEMPERATURE;  // 设置过温状态
      DJI_Motor->ERRORHandler.ErrorCount = 0;                  // 重置错误计数
    }
	}
  else
	{
    DJI_Motor->ERRORHandler.ErrorCount = 0;	  // 温度正常，重置错误计数
	}
}

/**
 * @brief  达妙电机错误处理函数
 * @param  *Damiao_Motor 达妙电机信息结构体指针
 * @retval None
 */
static void Damiao_Motor_ErrorHandler(Damiao_Motor_Info_Typedef *Damiao_Motor)
{
	Damiao_Motor->ERRORHandler.ErrorCount++;  // 错误计数增加
  if(Damiao_Motor->ERRORHandler.ErrorCount>200){
	 Damiao_Motor_DisEnable(Damiao_Motor->CANFrame.TxStdId);  // 错误次数过多，失能电机
	}
}
//------------------------------------------------------------------------------

/**
 * @brief  无符号整型转换为浮点型函数
 * @param  X_int 输入的无符号整型数值
 * @param  X_min 最小浮点值
 * @param  X_max 最大浮点值  
 * @param  Bits  数据位数
 * @retval float 转换后的浮点值
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// 根据范围和位数将无符号整型转换为浮点型 ///
    float span = X_max - X_min;      // 计算数值范围
    float offset = X_min;            // 偏移量
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

/**
 * @brief  达妙电机位置转角度函数(已注释)
 * @note   该函数用于将达妙电机的位置值转换为角度值
 * @param  Position 电机位置值(-12.5~12.5)
 * @retval float    对应的角度值(度)
 */
//static float Damiao_Motor_Postion_to_Angle(float Position){
//    if(Position>=0&&Position<=12.5) return (Position/12.5f*9)*180;      // 正向位置转换
//	  else if(Position<0&&Position>=-12.5)  return (Position/12.5f*9)*180;   // 负向位置转换
//}







