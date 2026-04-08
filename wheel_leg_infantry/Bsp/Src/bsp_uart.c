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

/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

#include "ctl_chassis.h"
#include "usart.h"
#include "remote_control.h"
#include "referee_info.h"
#include "pc_uart_ctrl.h"
#include "vofa.h"

/* Private variables ---------------------------------------------------------*/
/*云台-底盘通信*/
/* DMA双缓冲 */
__attribute__((section(".AXI_SRAM"))) uint8_t Mode_Buf[2][1];
uint8_t Usart_Mode = 0;

__attribute__((section(".AXI_SRAM")))
uint8_t UART6_Rx_Buf[2][50];  // 32字节缓冲区，确保能接收完整帧

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
 */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);
static void USART6_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);

/**
 * @brief  Configures the USART.
 * @param  None
 * @retval None
 */
void BSP_USART_Init(void)
{
  /* Starts the remote control multi_buffer DMA Transfer with interrupt enabled. */
  USART_RxDMA_MultiBufferStart(&huart3, (uint32_t *)&(huart3.Instance->DR), (uint32_t *)SBUS_MultiRx_Buf[0], (uint32_t *)SBUS_MultiRx_Buf[1], SBUS_RX_BUF_NUM);

  /* Starts the Referee multi_buffer DMA Transfer with interrupt enabled. */
  USART_RxDMA_MultiBufferStart(&huart1, (uint32_t *)&(huart1.Instance->DR), (uint32_t *)REFEREE_MultiRx_Buf[0], (uint32_t *)REFEREE_MultiRx_Buf[1], REFEREE_RXFRAME_LENGTH);

  /* 初始化云台-底盘通信DMA双缓冲接收（已被 PC 串口控制模块占用 USART6，暂时注释） */
  /* 如需恢复底盘通信，请注释下方 pc_uart_ctrl_init，并取消本段注释，
   * 同时将 PC_CTRL_DEFAULT_HUART 修改为其他可用串口。              */
  // USART_RxDMA_MultiBufferStart(&huart6,(uint32_t *)&(huart6.Instance->DR),
  // 								 (uint32_t *)UART6_Rx_Buf[0],
  // 								 (uint32_t *)UART6_Rx_Buf[1],
  // 								 FRAME_SIZE);
  USART6_RxDMA_MultiBufferStart(&huart6,
  								 (uint32_t *)&(huart6.Instance->DR),
  								 (uint32_t *)UART6_Rx_Buf[0],
  								 (uint32_t *)UART6_Rx_Buf[1],
  								 FRAME_SIZE);

  /* PC 串口控制模块初始化（默认使用 USART6，修改 PC_CTRL_DEFAULT_HUART 可切换串口）
   * 若切换到 USART6 以外的串口，需在下方 HAL_UARTEx_RxEventCallback 中
   * 为新串口添加 else-if 分支并调用 PC_Ctrl_RxHandler。               */
  // pc_uart_ctrl_init(PC_CTRL_DEFAULT_HUART);
}
//------------------------------------------------------------------------------

/**
 * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
 * @param  huart       pointer to a UART_HandleTypeDef structure that contains
 *                     the configuration information for the specified USART Stream.
 * @param  SrcAddress pointer to The source memory Buffer address
 * @param  DstAddress pointer to The destination memory Buffer address
 * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer
 * @param  DataLength The length of data to be transferred from source to destination
 * @retval none
 */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
  /* configuare the huart Reception Type TOIDLE */
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* configuare the huart Receicve Size */
  huart->RxXferSize = SBUS_RX_BUF_NUM;

  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
  } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address  */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
 * @brief  Starts USART6 fixed-length frame double-buffer DMA reception.
 * @param  huart            UART handle (USART6)
 * @param  SrcAddress       UART DR address
 * @param  DstAddress       DMA Memory0 address
 * @param  SecondMemAddress DMA Memory1 address
 * @param  DataLength       Frame length (FRAME_SIZE)
 * @retval None
 */
static void USART6_RxDMA_MultiBufferStart(UART_HandleTypeDef *huart, uint32_t *SrcAddress, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
  /* configuare the huart Reception Type TOIDLE */
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* USART6 fixed frame size should match DataLength */
  huart->RxXferSize = (uint16_t)DataLength;

  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Disable DMA */
  do
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
  } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* Configure the source memory Buffer address */
  huart->hdmarx->Instance->PAR = (uint32_t)SrcAddress;

  /* Configure the destination memory Buffer address */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;

  /* Configure DMA Stream destination address */
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* Configure the length of data to be transferred from source to destination */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* Enable double memory buffer */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
 * @brief  USER USART3 Reception Event Callback.
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Current memory buffer used is Memory 0 */
  if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
  {
    // Disable DMA
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
    /* reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

    if (Size == RC_FRAME_LENGTH)
    {
      SBUS_TO_RC(SBUS_MultiRx_Buf[0], &remote_ctrl);
    }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
    // Disable DMA
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

    /* reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);

    if (Size == RC_FRAME_LENGTH)
    {
      SBUS_TO_RC(SBUS_MultiRx_Buf[1], &remote_ctrl);
    }
  }
}
////------------------------------------------------------------------------------

/**
 * @brief  USER USART1 Reception Event Callback.
 * @param  huart UART handle
 * @param  Size  Number of data available in application reception buffer (indicates a position in
 *               reception buffer until which, data are available)
 * @retval None
 */
static void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Current memory buffer used is Memory 0 */
  if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
  {
    // Disable DMA
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR |= DMA_SxCR_CT;

    if (Size >= 10)
    {
      Referee_Frame_Update(REFEREE_MultiRx_Buf[0]);
      memset(REFEREE_MultiRx_Buf[0], 0, REFEREE_RXFRAME_LENGTH);
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx, REFEREE_RXFRAME_LENGTH);
    }
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
    // Disable DMA
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

    if (Size >= 10)
    {
      Referee_Frame_Update(REFEREE_MultiRx_Buf[1]);
      memset(REFEREE_MultiRx_Buf[1], 0, REFEREE_RXFRAME_LENGTH);
      /* reset the receive count */
      __HAL_DMA_SET_COUNTER(huart->hdmarx, REFEREE_RXFRAME_LENGTH);
    }
  }
}
/* UART6 Rx Cache Buffer for incomplete frames */
static uint8_t u6_rx_cache[50];
static uint16_t u6_rx_len = 0;

static void USER_USART6_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint8_t *rx_buf = NULL;

  /* Current memory buffer used is Memory 0 */
  if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
  {
    /* Disable DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
    rx_buf = UART6_Rx_Buf[0];
    /* reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, FRAME_SIZE);
  }
  /* Current memory buffer used is Memory 1 */
  else
  {
    /* Disable DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
    rx_buf = UART6_Rx_Buf[1];
    /* reset the receive count */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, FRAME_SIZE);
  }

  /* Append new data to cache */
  // Simple overflow protection
  if (u6_rx_len + Size > sizeof(u6_rx_cache)) {
       u6_rx_len = 0; // Clear buffer on overflow
  }

  if (u6_rx_len + Size <= sizeof(u6_rx_cache)) {
       memcpy(u6_rx_cache + u6_rx_len, rx_buf, Size);
       u6_rx_len += Size;
  }

  /* Process cached data */
  int start_idx = 0;
  while (start_idx < u6_rx_len) {
       // Check enough bytes for header check (need 2)
       if (start_idx + 1 >= u6_rx_len) {
            // Only 1 byte left. If it is AA, keep it (break loop and shift later).
            // If not, discard (increment start_idx).
            if (u6_rx_cache[start_idx] == UART_FRAME_HEADER1) {
                break;
            } else {
                start_idx++;
                continue;
            }
       }

       // Check header
       if (u6_rx_cache[start_idx] == UART_FRAME_HEADER1 &&
           u6_rx_cache[start_idx+1] == UART_FRAME_HEADER2)
       {
            if (start_idx + FRAME_SIZE <= u6_rx_len) {
                 chassis_parse_ctrl_cmd(&u6_rx_cache[start_idx]);
                 start_idx += FRAME_SIZE;
                 continue;
            } else {
                 // Waiting for rest of frame
                 break;
            }
       }

       start_idx++;
  }

  /* Shift remaining data to start of buffer */
  if (start_idx > 0) {
       uint16_t remaining = u6_rx_len > start_idx ? u6_rx_len - start_idx : 0;
       if (remaining > 0) {
            memmove(u6_rx_cache, &u6_rx_cache[start_idx], remaining);
       }
       u6_rx_len = remaining;
  }
}

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

  if (huart->Instance == USART3)
  {
    USER_USART3_RxHandler(huart, Size);
  }
  else if (huart->Instance == USART1)
  {
    USER_USART1_RxHandler(huart, Size);
  }
  else if (huart->Instance == USART6)
  {
    /* 恢复USART6用于底盘通信 */
    USER_USART6_RxHandler(huart, Size);
  }
  /* 若 PC_CTRL_DEFAULT_HUART 被修改为 USART6 以外的串口，在此新增分支，例如：*/
  // else if(huart->Instance == USART6) { PC_Ctrl_RxHandler(huart, Size); }
  /* reset the Reception Type */
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* Enalbe IDLE interrupt */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* Enable the DMA transfer for the receiver request */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* Enable DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------
