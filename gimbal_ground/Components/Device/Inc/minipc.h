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
#define MINIPC_ADDR_GIMBAL_ATTITUDE         0x13U
#define MINIPC_AUTO_AIM_FRAME_LENGTH        17U
#define MINIPC_GIMBAL_ATTITUDE_FRAME_LENGTH 16U

/**
 * @brief macro definition Number of MiniPC data to be sent
 */
#define MINIPC_SENDLENGTH     28U

/**
 * @brief macro definition Number of MiniPC to be receive
 */
#define MINIPC_REVCLENGTH     48U


/* Exported types ------------------------------------------------------------*/

/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information  for the MiniPC Receive Data.
 */
typedef struct 
{
  uint8_t header;
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum;
}MiniPC_SendPacket_Typedef;

/**
 * @brief typedef structure that contains the information for the MiniPC Transmit Data.
 */
typedef struct 
{
  uint8_t header;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum;
}MiniPC_ReceivePacket_Typedef;

/* restore byte alignment */
#pragma  pack()

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

typedef struct
{
  uint8_t header;
  uint8_t address;
  uint8_t length;
  float yaw;
  float pitch;
  float roll;
  uint8_t checksum;
} MiniPC_GimbalAttitudeFrame_Typedef;
#pragma pack()
/* Exported variables --------------------------------------------------------*/
/**
 * @brief structure that contains the information for the MiniPC Transmit Data.
 */
extern MiniPC_SendPacket_Typedef MiniPC_SendPacket;
/**
 * @brief structure that contains the information for the MiniPC Receive Data.
 */
extern MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket;

/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  Send the MiniPC frame Information according the USB CDC
  */
extern void MiniPC_SendFrameInfo(MiniPC_SendPacket_Typedef *SendPacket);
/**
  * @brief  Receive the MiniPC frame Information according the USB CDC
  */
extern void MiniPC_RecvFrameInfo(uint8_t* Buf, const uint32_t *Len);
bool MiniPC_DecodeAutoAimFrame(const uint8_t *buf, uint32_t len, MiniPC_AutoAimFrame_Typedef *frame);
bool MiniPC_SendGimbalAttitude(float yaw, float pitch, float roll);

#endif //DEVICE_MINIPC_H

