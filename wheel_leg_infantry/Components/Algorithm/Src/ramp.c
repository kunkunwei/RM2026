/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ramp.c
  * @brief          : 斜坡（ramp）函数
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待完善
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ramp.h"

/* Private define ------------------------------------------------------------*/
/**
  * @brief 计算浮点斜坡滤波器。
  * @param input: 滤波器输入值
  * @param target: 目标值
  * @param ramp: 斜坡步长（斜率）
  * @retval 滤波器输出值
  */
float f_Ramp_Calc(float input,float target,float ramp)
{
  float error = target - input;
  float output = input;

	if (error > 0){
        if (error > ramp){output += ramp;}   
        else{output += error;}
    }else{
        if (error < -ramp){output += -ramp;}
        else{output += error;}
    }

    return output;
}
//------------------------------------------------------------------------------


/**
  * @brief 计算浮点 Logistic 曲线。
  * @param x: 曲线输入值
  * @param k: 曲线斜率
  * @param x0: 曲线相位（偏移）
  * @note y = 1/(1+e^(-k*(x-x0)))
  *       k > 0: 1->0
  *       k < 0: 0->1
  * @retval 曲线输出值
  */
float f_LogisticCurves_Calc(float x , float k ,float x0)
{
	float y = 0.f;
	
	if(k == 0.f)return 1.f;
	
	y = 1/(1+pow(Euler_Number,(k*(x-x0))));
	
	return y;
}
//------------------------------------------------------------------------------


/**
  * @brief 根据 MovingAverage_Info_TypeDef 中的参数初始化移动平均滤波器。
  * @param MA: 指向包含移动平均滤波器信息的 MovingAverage_Info_TypeDef 结构体的指针。
  * @param length: 滤波缓冲区长度
  * @retval none
  */
void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length)
{

  MA->length = length;

  MA->filter_buff = malloc(sizeof(float)*MA->length);
  memset(MA->filter_buff,0,sizeof(float)*MA->length);

  if(MA->filter_buff == NULL)
  {
    return ;
  }

  MA->input = 0;
  MA->output = 0;

  MA->init = true;
}
//------------------------------------------------------------------------------


/**
  * @brief 计算浮点移动平均滤波器的输出。
  * @param MA: 指向包含移动平均滤波器信息的 MovingAverage_Info_TypeDef 结构体的指针。
  * @param input: 输入值
  * @retval 滤波器输出值
  */
float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input)
{
  if(MA->init != true)
  {
    return 0;
  }

  /* 将滤波缓冲区数据后移 */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->filter_buff[i+1] = MA->filter_buff[i];
  }

  /* 更新滤波输入 */
  MA->filter_buff[0] = input;

  /* 计算平均值 */
  for(uint16_t i = 0; i < MA->length-1; i++)
  {
      MA->sum += MA->filter_buff[i];
  }

  MA->output = (float)(MA->sum / MA->length);
	
	MA->sum = 0;

  return MA->output;
}
//------------------------------------------------------------------------------
