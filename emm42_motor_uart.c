// Created by kun on 25-7-12.
// Updated on 2025-07-13 for enhanced functionality, RTOS compatibility, and simplified parameters.

/**
 * @file    emm42_motor_uart.c
 * @brief   UART driver for Emm42 closed-loop stepper motor with multi-address support
 * @version 2.1
 * @date    2025-07-13
 */

#include "emm42_motor_uart.h"
#include "usart.h"
#include <string.h>

/**
 * @brief Checksum byte for UART communication
 */
#define CHECKSUM_BYTE 0x6B

/**
 * @brief UART transmission timeout (ms)
 */
#define TX_TIMEOUT_MS 100

/**
 * @brief UART reception timeout (ms)
 */
#define RX_TIMEOUT_MS 100

/**
 * @brief Default UART handle for motor communication
 */
#define em42_huart &huart3

/**
 * @brief Send a command to the motor
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param cmd Command data buffer
 * @param len Length of command data (max 10 bytes to fit 12-byte frame)
 * @retval true Success, false Failure or invalid length
 * @note Assumes em42_huart is initialized
 */
static bool Motor_SendCommand(uint8_t addr, uint8_t *cmd, uint8_t len)
{
  if (len > 10)
    return false; // Prevent buffer overflow (12 - addr - checksum)
  uint8_t frame[12] = {0};
  frame[0] = addr;
  memcpy(&frame[1], cmd, len);
  frame[len + 1] = CHECKSUM_BYTE;
  return (HAL_UART_Transmit(em42_huart, frame, len + 2, TX_TIMEOUT_MS) == HAL_OK);
}

/**
 * @brief Receive and validate motor response
 * @param addr Motor address (0–247, 0 for broadcast)
 * @param rx Buffer to store response
 * @param len Expected response length
 * @retval Motor_ResponseStatus Status of response (OK, ERROR, or INVALID)
 * @note For broadcast address (0), only address 1 responds; others may not
 */
static Motor_ResponseStatus Motor_ReceiveResponse(uint8_t addr, uint8_t *rx, uint8_t len)
{
  if (HAL_UART_Receive(em42_huart, rx, len, RX_TIMEOUT_MS) != HAL_OK)
    return RESPONSE_INVALID;
  if (rx[0] != addr || rx[len - 1] != CHECKSUM_BYTE)
    return RESPONSE_INVALID;
  if (len >= 2 && rx[1] == 0x02)
    return RESPONSE_OK;
  if (len >= 2 && rx[1] == 0xEE)
    return RESPONSE_ERROR;
  return RESPONSE_INVALID;
}

bool Motor_Enable(uint8_t addr, bool enable)
{
  if (addr > 247)
    return false;
  uint8_t cmd[] = {0xF3, enable ? 0x01 : 0x00};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_SetSubdivision(uint8_t addr, uint8_t subdiv)
{
  if (addr > 247 || subdiv == 0 || subdiv > 256)
    return false;
  uint8_t cmd[] = {0x84, subdiv};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_UnlockStallProtection(uint8_t addr)
{
  if (addr > 247)
    return false;
  uint8_t cmd[] = {0x0E, 0x52};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_SetSpeedDirection(uint8_t addr, int16_t speed, uint8_t acc)
{
  if (addr > 247 || acc > 255)
    return false;
  uint16_t abs_speed = (speed < 0) ? -speed : speed;
  if (abs_speed > 0x4FF)
    abs_speed = 0x4FF;                      // Max speed: 1279
  uint8_t direction = (speed >= 0) ? 1 : 0; // 1: clockwise, 0: counterclockwise
  uint8_t cmd[] = {
      0xF6,
      (uint8_t)(((direction & 0x0F) << 4) | ((abs_speed >> 8) & 0x0F)),
      (uint8_t)(abs_speed & 0xFF),
      acc};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_MoveRelative(uint8_t addr, int16_t speed, uint8_t acc, float angle)
{
  if (addr > 247 || acc > 255)
    return false;
  uint16_t abs_speed = (speed < 0) ? -speed : speed;
  if (abs_speed > 0x4FF)
    abs_speed = 0x4FF;                      // Max speed: 1279
  uint8_t direction = (speed >= 0) ? 1 : 0; // 1: clockwise, 0: counterclockwise
  // Convert angle to pulses (assuming 16 subdivisions): pulses = (angle * 65536) / 360
  uint32_t pulse = (uint32_t)((angle * 65536.0f) / 360.0f);
  uint8_t cmd[] = {
      0xFD,
      (uint8_t)(((direction & 0x0F) << 4) | ((abs_speed >> 8) & 0x0F)),
      (uint8_t)(abs_speed & 0xFF),
      acc,
      (uint8_t)((pulse >> 16) & 0xFF),
      (uint8_t)((pulse >> 8) & 0xFF),
      (uint8_t)(pulse & 0xFF)};
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  // Wait for acknowledgment (01 02 6B)
  uint8_t rx[3];
  Motor_ResponseStatus status = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  return (status == RESPONSE_OK);
}

bool Motor_EmergencyStop(uint8_t addr)
{
  if (addr > 247)
    return false;
  uint8_t cmd[] = {0xFD, 0x12, 0xFF, 0xFF, 0x00, 0x00, 0x00};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_ReadEnableStatus(uint8_t addr, uint8_t *status)
{
  if (addr > 247 || status == NULL)
    return false;
  uint8_t cmd[] = {0x3A};
  uint8_t rx[3];
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *status = rx[1];
  return true;
}

bool Motor_ReadEncoder(uint8_t addr, uint16_t *value)
{
  if (addr > 247 || value == NULL)
    return false;
  uint8_t cmd[] = {0x30};
  uint8_t rx[4];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *value = ((uint16_t)rx[1] << 8) | rx[2];
  return true;
}

bool Motor_ReadPosition(uint8_t addr, int32_t *pos)
{
  if (addr > 247 || pos == NULL)
    return false;
  uint8_t cmd[] = {0x36};
  uint8_t rx[6];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *pos = ((int32_t)rx[1] << 24) | ((int32_t)rx[2] << 16) | ((int32_t)rx[3] << 8) | rx[4];
  return true;
}

bool Motor_ReadPulseCount(uint8_t addr, int32_t *count)
{
  if (addr > 247 || count == NULL)
    return false;
  uint8_t cmd[] = {0x33};
  uint8_t rx[6];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *count = ((int32_t)rx[1] << 24) | ((int32_t)rx[2] << 16) | ((int32_t)rx[3] << 8) | rx[4];
  return true;
}

bool Motor_ReadError(uint8_t addr, int16_t *error)
{
  if (addr > 247 || error == NULL)
    return false;
  uint8_t cmd[] = {0x39};
  uint8_t rx[4];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *error = ((int16_t)rx[1] << 8) | rx[2];
  return true;
}

bool Motor_ReadStallFlag(uint8_t addr, uint8_t *flag)
{
  if (addr > 247 || flag == NULL)
    return false;
  uint8_t cmd[] = {0x3E};
  uint8_t rx[3];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *flag = rx[1];
  return true;
}

bool Motor_ReadAutoHomeStatus(uint8_t addr, uint8_t *status)
{
  if (addr > 247 || status == NULL)
    return false;
  uint8_t cmd[] = {0x3F};
  uint8_t rx[3];
  if (!Motor_SendCommand(addr, cmd, sizeof(cmd)))
    return false;
  Motor_ResponseStatus res = Motor_ReceiveResponse(addr, rx, sizeof(rx));
  if (res != RESPONSE_OK)
    return false;
  *status = rx[1];
  return true;
}

bool Motor_SetUartAddress(uint8_t addr, uint8_t new_addr)
{
  if (addr > 247 || new_addr == 0 || new_addr > 247)
    return false;
  uint8_t cmd[] = {0xAE, new_addr};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_StoreSpeedParameters(uint8_t addr)
{
  if (addr > 247)
    return false;
  uint8_t cmd[] = {0xFF, 0xC8};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

bool Motor_ClearSpeedParameters(uint8_t addr)
{
  if (addr > 247)
    return false;
  uint8_t cmd[] = {0xFF, 0xCA};
  return Motor_SendCommand(addr, cmd, sizeof(cmd));
}

float Motor_PositionToAngle(int32_t pos)
{
  return (float)(pos * 360) / 65536;
}

float Motor_ErrorToAngle(int16_t error)
{
  return (float)(error * 360) / 65536;
}