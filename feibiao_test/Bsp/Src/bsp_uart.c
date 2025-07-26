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
#include "usart.h"
#include "remote_control.h"
// #include "referee_info.h"

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  */
static void USART_RxDMA_MultiBufferStart(UART_HandleTypeDef *, uint32_t *, uint32_t *, uint32_t *, uint32_t);


/**
  * @brief  Configures the USART.
  * @param  None
  * @retval None
  */
void BSP_USART_Init(void)
{
  /* Starts the remote control multi_buffer DMA Transfer with interrupt enabled. */
	USART_RxDMA_MultiBufferStart(&huart3,(uint32_t *)&(huart3.Instance->DR),(uint32_t *)SBUS_MultiRx_Buf[0],(uint32_t *)SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);

	  /* Starts the Referee multi_buffer DMA Transfer with interrupt enabled. */
	// USART_RxDMA_MultiBufferStart(&huart6,(uint32_t *)&(huart6.Instance->DR),(uint32_t *)REFEREE_MultiRx_Buf[0],(uint32_t *)REFEREE_MultiRx_Buf[1],REFEREE_RXFRAME_LENGTH);
}
//------------------------------------------------------------------------------

/**
* @brief 启动具有中断使能的多缓冲 DMA 传输。
* @param  huart 指向包含指定 USART 流配置信息的 UART_HandleTypeDef 结构体的指针。
* @param  SrcAddress 源内存缓冲区地址的指针。
* @param  DstAddress 目标内存缓冲区地址的指针。
* @param  SecondMemAddress 在多缓冲区传输的情况下，指向第二个内存缓冲区地址的指针。
* @param  DataLength 从源传输到目标的数据长度。
* @retval 无。
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
  do{
      __HAL_DMA_DISABLE(huart->hdmarx);
  }while(huart->hdmarx->Instance->CR & DMA_SxCR_EN);

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
* @简要说明  用户 USART3 接收事件回调函数。
* @参数  huart UART 句柄
* @参数  Size 应用程序接收缓冲区中可用的数据数量（表示接收缓冲区中数据可用的位置）
* @返回值  无
 */
static void USER_USART3_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{
  // 先关闭DMA，防止数据竞争
  __HAL_DMA_DISABLE(huart->hdmarx);
  while(huart->hdmarx->Instance->CR & DMA_SxCR_EN) {
    // 等待DMA完全关闭
  }

  // 判断当前使用的缓冲区
  if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
  {
    // 切换到Memory 1
    huart->hdmarx->Instance->CR |= DMA_SxCR_CT;
    // 重装计数器
    __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
    // 数据长度正确才解析
    if(Size == RC_FRAME_LENGTH)
    {
      SBUS_TO_RC(SBUS_MultiRx_Buf[0],&remote_ctrl);
    }
  }
  else
  {
    // 切换到Memory 0
    huart->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
    // 重装计数器
    __HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM);
    if(Size == RC_FRAME_LENGTH)
    {
      SBUS_TO_RC(SBUS_MultiRx_Buf[1],&remote_ctrl);
    }
  }

  // 重新使能DMA，准备接收下一帧
  __HAL_DMA_ENABLE(huart->hdmarx);
}


/**
  * @brief  接收传输完成回调函数。
  * @param  huart  指向包含指定UART模块配置信息的UART_HandleTypeDef结构体指针。
  * @retval 无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{

	if(huart->Instance == USART3)
	{
		USER_USART3_RxHandler(huart,Size);
	}else if(huart->Instance == USART6)
	{
		// USER_USART1_RxHandler(huart,Size);
	}

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
