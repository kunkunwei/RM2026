/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : minipc.c
  * @brief          : minipc interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

#define MINIPC_FRAME_HEADER                 0x42U
#define MINIPC_ADDR_AUTO_AIM_CMD            0x12U
#define MINIPC_ADDR_GIMBAL_ATTITUDE         0x21U
#define MINIPC_AUTO_AIM_FRAME_LENGTH        17U
#define MINIPC_CMD_FRAME_LENGTH             13U

/* Exported types ------------------------------------------------------------*/

/* cancel byte alignment */
#pragma  pack(1)

/* 26赛季自瞄结构体*/
typedef union {

  struct _control_flags {
    uint8_t auto_aim : 1;        // 自动瞄准标志
    uint8_t auto_shoot : 1;      // 自动射击标志
    uint8_t exit_auto_aim : 1;   // 退出自动瞄准标志
    uint8_t exit_auto_shoot : 1; // 退出自动射击标志
    uint8_t reserved : 4;        // 保留位
  } bits;
  uint8_t all_flags;

} Control_Flag_t;

typedef union
{
  float value;
  uint8_t bytes[4];
} MiniPC_FloatUnion_t;

#pragma pack(1)
typedef struct
{
  uint8_t header;
  uint8_t address;
  uint8_t length;
  float yaw;
  float pitch;
  float depth;
  Control_Flag_t control_flag;
  uint8_t checksum;
} MiniPC_AutoAimFrame_Typedef;
#pragma pack()

/* Exported functions prototypes ---------------------------------------------*/
bool MiniPC_DecodeAutoAimFrame(const uint8_t *buf, uint32_t len, MiniPC_AutoAimFrame_Typedef *frame);
bool MiniPC_Sendgimbal(float yaw, float pitch);

#endif //DEVICE_MINIPC_H

