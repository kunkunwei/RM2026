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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAMP_H
#define RAMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information  for the first order lowpass filter.
 */
typedef struct
{
    bool init;             /*!< 初始化标志 */
    float *filter_buff;    /*!< 滤波缓冲区浮点数组指针 */
    uint16_t length;       /*!< 缓冲区长度 */
    float input;           /*!< 输入值 */
    float sum;             /*!< 求和变量 */
    float output;          /*!< 输出值 */
}MovingAverage_Info_TypeDef;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief 计算浮点斜坡滤波器。
  */
extern float f_Ramp_Calc(float input,float target,float ramp);
/**
  * @brief 计算浮点 Logistic 曲线。
  */
extern float f_LogisticCurves_Calc(float x , float k ,float x0);
/**
  * @brief 根据 MovingAverage_Info_TypeDef 中的参数初始化移动平均滤波器。
  */
extern void MovingAverage_Init(MovingAverage_Info_TypeDef *MA,uint16_t length);
/**
  * @brief 更新浮点移动平均滤波器并返回输出。
  */
extern float MovingAverage_Update(MovingAverage_Info_TypeDef *MA,float input);

#endif //RAMP_H
