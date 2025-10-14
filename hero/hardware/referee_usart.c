#include "referee_usart.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

//UART6_Tx PG14    UART6_Rx PG9
void referee_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 在HAL库中，UART和GPIO的基本初始化已经在CubeMX生成的代码中完成
    // 这里只需要重新配置UART参数和启动DMA接收

    // 设置UART参数
    huart6.Init.BaudRate = 115200;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    // 重新初始化UART
    HAL_UART_DeInit(&huart6);
    HAL_UART_Init(&huart6);

    // 使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

    // 配置NVIC
    HAL_NVIC_SetPriority(USART6_IRQn, REFEREE_NVIC, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

    // 启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rx1_buf, dma_buf_num);

    // 禁用DMA接收半满中断
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void referee_restart(uint16_t dma_buf_num)
{
    // 停止UART和DMA
    HAL_UART_DMAStop(&huart6);

    // 清除IDLE标志
    __HAL_UART_CLEAR_IDLEFLAG(&huart6);

    // 清除DMA标志
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t *)huart6.pRxBuffPtr, dma_buf_num);

    // 禁用DMA接收半满中断
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}
