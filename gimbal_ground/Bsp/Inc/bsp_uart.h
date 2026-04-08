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
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "config.h" /* 提供 USE_SBUS_PROTOCOL 宏定义 */

  /* Exported defines -----------------------------------------------------------*/
  /**
   * @brief Remote control protocol selection
   * @note  USE_SBUS_PROTOCOL 在 config.h 中统一控制
   *        SBUS: 需将 USART3 配置为 100K, 8E2
   *        DBUS: 需将 USART3 配置为 100K, 8N1
   */
  /* Exported functions prototypes ---------------------------------------------*/
  /**
   * @brief  Configures the USART.
   */
  extern void BSP_USART_Init(void);

#endif // BSP_UART_H
