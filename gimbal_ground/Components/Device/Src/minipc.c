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
#include "crc.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
/**
 * @brief Buffer of MiniPC data to be sent
 */
uint8_t MiniPC_SendBuf[MINIPC_SENDLENGTH];

/**
 * @brief structure that contains the information for the MiniPC Receive Data.
 */
MiniPC_ReceivePacket_Typedef MiniPC_ReceivePacket = {
  .header = 0x42,
};
/**
 * @brief structure that contains the information for the MiniPC Transmit Data.
 */
MiniPC_SendPacket_Typedef MiniPC_SendPacket = {
    .header = 0x5A,
};

static uint8_t MiniPC_FrameChecksum(const uint8_t *buf, uint8_t len)
{
  uint8_t checksum = 0;

  if (buf == NULL || len < 2U)
  {
    return 0U;
  }

  for (uint8_t i = 0; i < (uint8_t)(len - 2U); i++)
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

/**
  * @brief  Send the MiniPC frame Information according the USB CDC
  * @param  SendPacket: pointer to MiniPC_SendPacket_Typedef structure that 
  *         contains the information for the MiniPC Transmit Data.
  * @retval none
  */
void MiniPC_SendFrameInfo(MiniPC_SendPacket_Typedef *SendPacket)
{
  /* calculate the crc16 */
  SendPacket->checksum = get_CRC16_check_sum((uint8_t *)SendPacket,MINIPC_SENDLENGTH-2,0xffff);

  /* store the MiniPC data to be sent */
  memcpy(MiniPC_SendBuf,(uint8_t *)SendPacket,MINIPC_SENDLENGTH);

  /* USB Send data */
  CDC_Transmit_FS(MiniPC_SendBuf,MINIPC_SENDLENGTH);
}
//------------------------------------------------------------------------------

/**
  * @brief  Receive the MiniPC frame Information according the USB CDC
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval none
  */
void MiniPC_RecvFrameInfo(uint8_t* Buf, const uint32_t *Len)
{
  /* Judge the crc16 */
  if(verify_CRC16_check_sum(Buf,*Len) != true)
  {
    return ;
  }

  /* judge the frame header */
  if(Buf[0] == MiniPC_ReceivePacket.header)
  {
    /* store the receive data */
    memcpy(&MiniPC_ReceivePacket,Buf,*Len);
  }
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
  frame->pitch = MiniPC_UnpackFloat(&buf[7]);
  frame->depth = MiniPC_UnpackFloat(&buf[11]);
  frame->control_flag.all_flags = buf[15];
  frame->checksum = buf[16];

  return true;
}

bool MiniPC_SendGimbalAttitude(float yaw, float pitch, float roll)
{
  uint8_t tx_buf[MINIPC_GIMBAL_ATTITUDE_FRAME_LENGTH] = {0};

  tx_buf[0] = MINIPC_FRAME_HEADER;
  tx_buf[1] = MINIPC_ADDR_GIMBAL_ATTITUDE;
  tx_buf[2] = MINIPC_GIMBAL_ATTITUDE_FRAME_LENGTH;

  MiniPC_PackFloat(&tx_buf[3], yaw);
  MiniPC_PackFloat(&tx_buf[7], pitch);
  MiniPC_PackFloat(&tx_buf[11], roll);
  tx_buf[15] = MiniPC_FrameChecksum(tx_buf, MINIPC_GIMBAL_ATTITUDE_FRAME_LENGTH);

  return CDC_Transmit_FS(tx_buf, MINIPC_GIMBAL_ATTITUDE_FRAME_LENGTH) == USBD_OK;
}

