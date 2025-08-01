/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  //others
#include <stdio.h>
#include "cmsis_os.h"
#include "remote_control.h"
  //matlab
  // #include "leg_pos.h"
  // #include "leg_spd.h"
#include "lqr_k.h"
  // #include "leg_conv.h"
  //bsp
// #include "bsp_can.h"
#include "bsp_mcu.h"
// #include "emm42_motor_uart.h"
  //tasks
  // #include "INS_Task.h"
  // #include "Ros_task.h"

  // #include "Chassis_Task.h"
  //device
  // #include "usb.h"

// #include "mymotor.h"
// #include "vofa.h" 必须注释
// #include "stepper_pwm_control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
  typedef unsigned char bool_t;
  typedef float fp32;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
 // extern osSemaphoreId can_cmd_semHandle;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_9
#define LED_R_GPIO_Port GPIOF
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOF
#define Motor2_dir_Pin GPIO_PIN_1
#define Motor2_dir_GPIO_Port GPIOB
#define Motor2_en_Pin GPIO_PIN_2
#define Motor2_en_GPIO_Port GPIOB
#define Motor1_dir_Pin GPIO_PIN_10
#define Motor1_dir_GPIO_Port GPIOE
#define Motor1_en_Pin GPIO_PIN_11
#define Motor1_en_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
