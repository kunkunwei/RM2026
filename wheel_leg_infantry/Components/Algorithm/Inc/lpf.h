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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief 包含一阶低通滤波器信息的结构体类型。
 */
typedef struct
{
    bool Initialized;     /*!< 初始化标志 */
    float input;          /*!< 输入值 */
    float output;         /*!< 输出值 */
    float alpha;          /*!< 滤波系数 */
    float frame_period;   /*!< 帧周期 */
}LowPassFilter1p_Info_TypeDef;

/**
 * @brief 包含二阶低通滤波器信息的结构体类型。
 */
typedef struct 
{
    bool Initialized;  /*!< 初始化标志 */
    float input;       /*!< 输入值 */
    float output[3];   /*!< 输出数组 */
    float alpha[3];    /*!< 滤波系数数组 */
}LowPassFilter2p_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief 根据 LowPassFilter1p_Info_TypeDef 中的参数初始化一阶低通滤波器。
  */
extern void LowPassFilter1p_Init(LowPassFilter1p_Info_TypeDef *lpf,float alpha,float frame_period);
/**
  * @brief 根据 LowPassFilter1p_Info_TypeDef 更新一阶低通滤波器。
  */
extern float LowPassFilter1p_Update(LowPassFilter1p_Info_TypeDef *lpf,float input);
/**
  * @brief 根据 LowPassFilter2p_Info_TypeDef 中的参数初始化二阶低通滤波器。
  */
extern void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
/**
  * @brief 根据 LowPassFilter2p_Info_TypeDef 更新二阶低通滤波器。
  */
extern float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);

#endif //LOWPASS_FILTER_H
