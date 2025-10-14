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

/* 头文件包含 ------------------------------------------------------------------*/
#include "../Inc/motor.h"
#include "bsp_can.h"



 uint8_t DamiaoMotorAble_FramInfo[8];
 uint8_t DamiaoMotor2Able_FramInfo[8];

//};
/* 私有函数声明 ---------------------------------------------------------------*/
/**
  * @brief  编码器(0-8192)转换为累计角度(3.4E38)
  */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *,float ,uint16_t );
/**
  * @brief  编码器(0-8192)转换为角度(-180~180)
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *,float ,uint16_t );
/** 
  * @brief  判断DJI电机状态
  */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *);

static void Damiao_Motor_ErrorHandler(Damiao_Motor_Info_Typedef *Damiao_Motor);
/**
  * @brief  更新DJI电机信息
  * @param  StdId  指向标准标识符的指针
  * @param  rxBuf  指向CAN接收数据的指针
  * @param  DJI_Motor 指向DJI_Motor_Info_t结构体的指针
  * @retval 无
  */
void DJI_Motor_Info_Update(uint32_t *StdId, uint8_t *rxBuf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* 检查StdId */
	if(*StdId != DJI_Motor->CANFrame.RxStdId) return;
	
	/* 解析通用电机数据 */
	DJI_Motor->Data.temperature = rxBuf[6];
	DJI_Motor->Data.encoder  = ((int16_t)rxBuf[0] << 8 | (int16_t)rxBuf[1]);
	DJI_Motor->Data.velocity = ((int16_t)rxBuf[2] << 8 | (int16_t)rxBuf[3]);
	DJI_Motor->Data.current  = ((int16_t)rxBuf[4] << 8 | (int16_t)rxBuf[5]);
	/* 判断电机错误 */
	DJI_Motor_ErrorHandler(DJI_Motor);

  /* 更新发送帧ID和索引 */
  if(DJI_Motor->Data.Initialized != true)
  {
    if(DJI_Motor->CANFrame.RxStdId > DJI_RxFrame_MIDDLE)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_HIGH;
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_RxFrame_MIDDLE - 0x01U);
    }
    else if(DJI_Motor->CANFrame.RxStdId > DJI_TxFrame_LOW)
    {
      DJI_Motor->CANFrame.TxStdId = DJI_TxFrame_LOW; 
      DJI_Motor->CANFrame.FrameIndex = 2*(DJI_Motor->CANFrame.RxStdId - DJI_TxFrame_LOW - 0x01U);
    }
  }
	
	/* 编码器转换为累计角度 */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,1.f,8192);
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,3591.f/187.f,8192);
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.angle = encoder_to_anglesum(&DJI_Motor->Data,36.f,8192);
		break;
		
		default:break;
	}
}
//------------------------------------------------------------------------------
/**
  * @brief  更新大妙电机信息
  * @param  StdId  指向标准标识符的指针
  * @param  rxBuf  指向CAN接收数据的指针
  * @param  RMD_Motor 指向RMD_L9025_Info_Typedef结构体的指针
  * @retval 无
  */
 
void Damiao_Motor_Info_Update(uint8_t *rxBuf,Damiao_Motor_Info_Typedef *Damiao_Motor)
{
	/* 判断StdId */
  	Damiao_Motor->ID = rxBuf[0]&0x0F;

	 if(Damiao_Motor->ID != Damiao_Motor->CANFrame.RxStdId) return;

	  Damiao_Motor->Data.State = rxBuf[0]>>4;
		Damiao_Motor->Data.P_int = ((uint16_t)(rxBuf[1]) <<8) | ((uint16_t)(rxBuf[2]));
		Damiao_Motor->Data.V_int = ((uint16_t)(rxBuf[3]) <<4) | ((uint16_t)(rxBuf[4])>>4);
		Damiao_Motor->Data.T_int = ((uint16_t)(rxBuf[4]&0xF) <<8) | ((uint16_t)(rxBuf[5]));

		Damiao_Motor->Data.Torque=  uint_to_float(Damiao_Motor->Data.T_int,-45,45,12);
		Damiao_Motor->Data.Position=uint_to_float(Damiao_Motor->Data.P_int,-3.141593,3.141593,16);
		Damiao_Motor->Data.Velocity=uint_to_float(Damiao_Motor->Data.V_int,-50,50,12);

   	//if(fabs(Damiao_Motor->Data.Velocity)<0.015) Damiao_Motor->Data.Velocity = 0;
		Damiao_Motor->Data.Temperature_MOS   = (float)(rxBuf[6]);
		Damiao_Motor->Data.Temperature_Rotor = (float)(rxBuf[7]);
		Damiao_Motor->Data.Angle =  Damiao_Motor->Data.Position*Rad_to_angle;
    
  // 位置修正
  if(Damiao_Motor->CANFrame.RxStdId == 0x01){
    Damiao_Motor->Data.Position -= 0.689898f;
    Damiao_Motor->Data.Position = 3.141593/2 - Damiao_Motor->Data.Position;
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x02){
    Damiao_Motor->Data.Position += 0.504883f;
    Damiao_Motor->Data.Position = 3.141593/2 - Damiao_Motor->Data.Position;
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x03){
    Damiao_Motor->Data.Position += 1.144236f;
    Damiao_Motor->Data.Position += 3.141593/2;
  }
  else if(Damiao_Motor->CANFrame.RxStdId == 0x03){
    Damiao_Motor->Data.Position += 0.473602f;
    Damiao_Motor->Data.Position += 3.141593/2;
  }

	if(Damiao_Motor->Data.State!=0){
   Damiao_Motor_ErrorHandler(Damiao_Motor);
	}
}


//------------------------------------------------------------------------------

/**
  * @brief  编码器(0-8192)转换为累计角度(3.4E38)
  * @param  *Info        指向Motor_GeneralInfo_Typedef结构体的指针
  * @param  torque_ratio 电机扭矩比
  * @param  MAXencoder   编码器最大值
  * @retval 累计角度
  */
static float encoder_to_anglesum(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{
  float res1 = 0,res2 =0;
  
  if(Info == NULL) return 0;
  
  /* 判断电机是否初始化 */
  if(Info->Initialized != true)
  {
    /* 更新上一次编码器值 */
    Info->last_encoder = Info->encoder;

    /* 重置角度 */
    Info->angle = 0;

    /* 设置初始化标志 */
    Info->Initialized = true;
  }
  
  /* 获取可能的最小编码器误差 */
  if(Info->encoder < Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder + MAXencoder;
  }
  else if(Info->encoder > Info->last_encoder)
  {
      res1 = Info->encoder - Info->last_encoder - MAXencoder;
  }
  res2 = Info->encoder - Info->last_encoder;
  
  /* 更新上一次编码器值 */
  Info->last_encoder = Info->encoder;
  
  /* 编码器数据转换为累计角度 */
	if(fabsf(res1) > fabsf(res2))
	{
		Info->angle += (float)res2/(MAXencoder*torque_ratio)*360.f;
	}
	else
	{
		Info->angle += (float)res1/(MAXencoder*torque_ratio)*360.f;
	}
  
  return Info->angle;
}
//------------------------------------------------------------------------------

/**
  * @brief  浮点数循环约束
  * @param  Input    输入变量
  * @param  minValue 最小值
  * @param  maxValue 最大值
  * @retval 约束后的变量
  */
static float f_loop_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }
  
  float len = maxValue - minValue;    

  if (Input > maxValue)
  {
      do{
          Input -= len;
      }while (Input > maxValue);
  }
  else if (Input < minValue)
  {
      do{
          Input += len;
      }while (Input < minValue);
  }
  return Input;
}
//------------------------------------------------------------------------------

/**
  * @brief  编码器(0-8192)转换为角度(-180~180)
  * @param  *Info        指向Motor_GeneralInfo_Typedef结构体的指针
  * @param  torque_ratio 电机扭矩比
  * @param  MAXencoder   编码器最大值
  * @retval 角度
  */
float encoder_to_angle(Motor_GeneralInfo_Typedef *Info,float torque_ratio,uint16_t MAXencoder)
{	
  float encoder_err = 0.f;
  
  /* 检查电机是否初始化 */
  if(Info->Initialized != true)
  {
    /* 更新上一次编码器值 */
    Info->last_encoder = Info->encoder;

    /* 重置角度 */
    Info->angle = 0;

    /* 设置初始化标志 */
    Info->Initialized = true;
  }
  
  encoder_err = Info->encoder - Info->last_encoder;
  
  /* 0 -> MAXencoder */		
  if(encoder_err > MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err - MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  /* MAXencoder-> 0 */		
  else if(encoder_err < -MAXencoder*0.5f)
  {
    Info->angle += (float)(encoder_err + MAXencoder)/(MAXencoder*torque_ratio)*360.f;
  }
  else
  {
    Info->angle += (float)(encoder_err)/(MAXencoder*torque_ratio)*360.f;
  }
  
  /* 更新上一次编码器值 */
  Info->last_encoder = Info->encoder;
  
  /* 循环约束 */
  Info->angle = f_loop_constrain(Info->angle,-180.f,180.f);

  return Info->angle;
}
//------------------------------------------------------------------------------

/** 
  * @brief  判断DJI电机状态
  * @param  *DJI_Motor 指向DJI_Motor_Info_Typedef结构体的指针
  * @retval 无
  */
static void DJI_Motor_ErrorHandler(DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* 判断DJI电机温度 */
	if(DJI_Motor->Data.temperature > 80)
	{
    DJI_Motor->ERRORHandler.ErrorCount++;

    if(DJI_Motor->ERRORHandler.ErrorCount > 200)
    {
      DJI_Motor->ERRORHandler.Status = MOTOR_OVER_TEMPERATURE;
      DJI_Motor->ERRORHandler.ErrorCount = 0;
    }
	}
  else
	{
    DJI_Motor->ERRORHandler.ErrorCount = 0;	
	}
}

/**
 * @brief 大妙电机错误处理
 */
static void Damiao_Motor_ErrorHandler(Damiao_Motor_Info_Typedef *Damiao_Motor)
{
	Damiao_Motor->ERRORHandler.ErrorCount++;
  if(	Damiao_Motor->ERRORHandler.ErrorCount>200){
	 Damiao_Motor_DisEnable(Damiao_Motor->CANFrame.TxStdId);
	}
}
//------------------------------------------------------------------------------
/**
 * @brief 无符号整型转浮点型
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// 将无符号整型转换为浮点型，给定范围和位数 ///
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}
//static float Damiao_Motor_Postion_to_Angle(float Position){
//    if(Position>=0&&Position<=12.5) return (Position/12.5f*9)*180;
//	  else if(Position<0&&Position>=-12.5)  return (Position/12.5f*9)*180
//}




