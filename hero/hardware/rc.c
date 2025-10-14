#include "rc.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    /* 在HAL库中，UART和DMA的基本初始化已经在CubeMX生成的代码中完成
       这里只需要配置并启动DMA接收 */

    // 设置UART为偶校验，100K波特率
    huart3.Init.BaudRate = 100000;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_EVEN;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.Mode = UART_MODE_RX;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    // 重新初始化UART，应用新的配置
    HAL_UART_DeInit(&huart3);
    HAL_UART_Init(&huart3);

    // 启用IDLE中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 启动DMA接收
    // DMA双缓冲模式，交替接收遥控器数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx1_buf, dma_buf_num);

    // 禁用DMA接收半满中断，我们只需要使用完成中断
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

// 添加HAL库的UART空闲中断回调函数
void USART3_IRQHandler(void)
{
    if(huart3.Instance == USART3)
    {
        // 检查是否为IDLE中断
        if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
        {
            // 清除IDLE标志位
            __HAL_UART_CLEAR_IDLEFLAG(&huart3);

            // 停止当前DMA传输
            HAL_UART_DMAStop(&huart3);

            // 计算接收到的数据长度
            uint32_t data_length = RC_FRAME_LENGTH - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

            // 判断接收到的数据长度是否正确
            if(data_length == RC_FRAME_LENGTH)
            {
                // 处理遥控器数据，可以在这里添加解析代码
                // ...
            }

            // 重新启动DMA接收
            HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)&huart3.pRxBuffPtr, RC_FRAME_LENGTH);
            __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
        }
    }

    // 调用HAL库的中断处理函数
    HAL_UART_IRQHandler(&huart3);
}
