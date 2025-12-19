/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lowpass_filter.c
  * @brief          : 低通滤波器
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : 待完善
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lpf.h"

/**
  * @brief 根据 LowPassFilter1p_Info_TypeDef 中的参数初始化一阶低通滤波器。
  * @param lpf: 指向 LowPassFilter1p_Info_TypeDef 结构体的指针，包含一阶低通滤波器的信息。
  * @param alpha: 滤波系数
  * @param frame_period: 帧周期
  * @retval none
  */
void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *lpf,float alpha,float frame_period)
{
  lpf->alpha = alpha;
  lpf->frame_period = frame_period;
  lpf->input = 0;
  lpf->output = 0;
}
//------------------------------------------------------------------------------


/**
  * @brief 根据 LowPassFilter1p_Info_TypeDef 更新一阶低通滤波器。
  * @param kf: 指向 LowPassFilter1p_Info_TypeDef 结构体的指针，包含一阶低通滤波器的信息。
  * @param input: 滤波器输入
  * @retval 低通滤波器输出
  */
float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input)
{
  lpf->input = input;

  if(lpf->Initialized == false)
  {
    lpf->output = lpf->input;
    lpf->Initialized = true;
  }

  lpf->output = lpf->alpha / (lpf->alpha + lpf->frame_period) * lpf->output 
              + lpf->frame_period / (lpf->alpha + lpf->frame_period) * lpf->input;

  return lpf->output;
}
//------------------------------------------------------------------------------


/**
  * @brief 根据 LowPassFilter2p_Info_TypeDef 中的参数初始化二阶低通滤波器。
  * @param lpf: 指向 LowPassFilter2p_Info_TypeDef 结构体的指针，包含二阶低通滤波器的信息。
  * @param alpha: 滤波系数数组
  * @param frame_period: 帧周期（未使用）
  * @retval none
  */
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3])
{
  memcpy(lpf->alpha,alpha,sizeof(lpf->alpha));
  lpf->input = 0;
  memset(lpf->output,0,sizeof(lpf->output));
}
//------------------------------------------------------------------------------


/**
  * @brief 根据 LowPassFilter2p_Info_TypeDef 更新二阶低通滤波器。
  * @param kf: 指向 LowPassFilter2p_Info_TypeDef 结构体的指针，包含二阶低通滤波器的信息。
  * @param input: 滤波器输入
  * @retval 低通滤波器输出（输出数组的第三个元素）
  */
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input)
{
	lpf->input = input;
  
  if(lpf->Initialized == false)
  {
    lpf->output[0] = lpf->input;
    lpf->output[1] = lpf->input;
    lpf->output[2] = lpf->input;
    lpf->Initialized = true;
  }
  
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[1] * lpf->output[0] + lpf->alpha[0] * lpf->output[1] + lpf->alpha[2] * lpf->input;

	return lpf->output[2];
}
//------------------------------------------------------------------------------
