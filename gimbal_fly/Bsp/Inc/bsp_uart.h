/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_uart.c
  * @brief          : bsp uart functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_UART_H
#define BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

  /* Exported defines -----------------------------------------------------------*/
  /**
   * @brief Remote control protocol selection
   * @note  Define USE_SBUS_PROTOCOL to use SBUS protocol (FlySky i6X - FS-IA6B)
   *        Comment it out to use DBUS protocol (DJI DT7 - DR16)
   */
  // //使用SBUS协议的时候，需要更改STM32CUBEMX里面的USART3配置，100K，8E2
  // //使用DBUS协议的时候，需要更改STM32CUBEMX里面的USART3配置，100K，8N1
  #define USE_SBUS_PROTOCOL  //解开这个注释来使用SBUS协议，否则使用DBUS协议
/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Configures the USART.
  */
extern void BSP_USART_Init(void);

#endif //BSP_UART_H
