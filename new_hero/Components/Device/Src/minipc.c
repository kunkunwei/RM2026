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

/* Includes ------------------------------------------------------------------*/
#include "minipc.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/

static uint8_t MiniPC_FrameChecksum(const uint8_t *buf, uint8_t len)
{
  uint8_t checksum = 0;

  if (buf == NULL || len < 2U)
  {
    return 0U;
  }

  for (uint8_t i = 0; i < (uint8_t)(len - 1U); i++)
  {
    checksum += buf[i];
  }

  return checksum;
}

static void MiniPC_PackFloat(uint8_t *dst, float value)
{
  MiniPC_FloatUnion_t data = {.value = value};

  memcpy(dst, data.bytes, sizeof(data.bytes));
}

static float MiniPC_UnpackFloat(const uint8_t *src)
{
  MiniPC_FloatUnion_t data = {0};

  memcpy(data.bytes, src, sizeof(data.bytes));
  return data.value;
}

//------------------------------------------------------------------------------

bool MiniPC_DecodeAutoAimFrame(const uint8_t *buf, uint32_t len, MiniPC_AutoAimFrame_Typedef *frame)
{
  if (buf == NULL || frame == NULL || len != MINIPC_AUTO_AIM_FRAME_LENGTH)
  {
    return false;
  }

  if (buf[0] != MINIPC_FRAME_HEADER || buf[1] != MINIPC_ADDR_AUTO_AIM_CMD || buf[2] != MINIPC_AUTO_AIM_FRAME_LENGTH)
  {
    return false;
  }

  if (MiniPC_FrameChecksum(buf, (uint8_t)len) != buf[len - 1U])
  {
    return false;
  }

  frame->header = buf[0];
  frame->address = buf[1];
  frame->length = buf[2];
  frame->yaw = MiniPC_UnpackFloat(&buf[3]);
  frame->pitch =- MiniPC_UnpackFloat(&buf[7]);
  frame->depth = MiniPC_UnpackFloat(&buf[11]);
  frame->control_flag.all_flags = buf[15];
  frame->checksum = buf[16];

  return true;
}

bool MiniPC_Sendgimbal(float yaw, float pitch)
{
  /* Frame: 0x42 0x21 13 | yaw | pitch | 0xFF | checksum(sum of first 12 bytes) */
  uint8_t tx_buf[MINIPC_CMD_FRAME_LENGTH] = {0};

  tx_buf[0] = MINIPC_FRAME_HEADER;
  tx_buf[1] = MINIPC_ADDR_GIMBAL_ATTITUDE;
  tx_buf[2] = MINIPC_CMD_FRAME_LENGTH;

  MiniPC_PackFloat(&tx_buf[3], yaw);
  MiniPC_PackFloat(&tx_buf[7], pitch);
  tx_buf[11] = 0xFF;
  tx_buf[12] = MiniPC_FrameChecksum(tx_buf, MINIPC_CMD_FRAME_LENGTH);

  return CDC_Transmit_FS(tx_buf, MINIPC_CMD_FRAME_LENGTH) == USBD_OK;
}
