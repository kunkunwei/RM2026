#include "Interface_UART.h"
#include "main_cpp.h"
#include "DBus.h"
#include "VT03.h"
#include "Referee.h"

using namespace BSP;

static uint8_t UART3_DMA_BUF[2][UART3_RX_BUF_LEN];
static uint8_t UART6_DMA_BUF[2][UART6_RX_BUF_LEN];
static uint8_t UART1_DMA_BUF[2][UART1_RX_BUF_LEN];

static void UART3_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    DMA_HandleTypeDef hdma_usart3_rx = *(huart3.hdmarx);

    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);//使能DMA串口接收
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能空闲中断

    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }//等待失能DMA完成

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR); //串口接收寄存器地址
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);//内存缓冲区1
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);//内存缓冲区2

    hdma_usart3_rx.Instance->NDTR = dma_buf_num; //数据长度
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM); //使能双缓冲区
    __HAL_DMA_ENABLE(&hdma_usart3_rx);//使能DMA
}

/*
static void UART3_unable()
{
    __HAL_UART_DISABLE(&huart3);
}
static void UART3_restart(uint16_t dma_buf_num)
{
    DMA_HandleTypeDef hdma_usart3_rx = *(huart3.hdmarx);

    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}
*/

static void UART6_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    DMA_HandleTypeDef hdma_usart6_rx = *(huart6.hdmarx);
    DMA_HandleTypeDef hdma_usart6_tx = *(huart6.hdmatx);

    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);//使能空闲中断

    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }//等待失能DMA完成

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);//清除DMA中断标志
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);//串口接收寄存器地址
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);//内存缓冲区1
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);//内存缓冲区2

    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);//数据长度
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM); //使能双缓冲区
    __HAL_DMA_ENABLE(&hdma_usart6_rx);//使能DMA


    //串口发送DMA配置
    __HAL_DMA_DISABLE(&hdma_usart6_tx);
    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }//等待失能DMA完成
    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);//串口发送寄存器地址
}


static void UART1_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    DMA_HandleTypeDef hdma_usart1_rx = *(huart1.hdmarx);

    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);//使能DMA串口接收
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能空闲中断

    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }//等待失能DMA完成

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR); //串口接收寄存器地址
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);//内存缓冲区1
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);//内存缓冲区2

    hdma_usart1_rx.Instance->NDTR = dma_buf_num; //数据长度
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM); //使能双缓冲区
    __HAL_DMA_ENABLE(&hdma_usart1_rx);//使能DMA
}


void BSP::UART_UserInit(){

    Device::VT03::getInstance(); //初始化VT03
    Device::DBus::getInstance(); //初始化Referee
    Device::Referee::getInstance(); //初始化DBus

    UART6_Init(UART6_DMA_BUF[0],UART6_DMA_BUF[1],UART6_RX_BUF_LEN);
    UART3_Init(UART3_DMA_BUF[0],UART3_DMA_BUF[1],UART3_RX_BUF_LEN);
    UART1_Init(UART1_DMA_BUF[0],UART1_DMA_BUF[1],UART1_RX_BUF_LEN);
}


extern "C" void USART1_IDEL_IRQHandler(UART_HandleTypeDef *huart){
    static uint16_t this_time_rx_len = 0;
    static uint32_t error_count = 0;
    static uint32_t success_count = 0;
    DMA_HandleTypeDef* hdma_usart1_rx = huart->hdmarx;

    __HAL_DMA_DISABLE(hdma_usart1_rx); //失能DMA
    this_time_rx_len = UART1_RX_BUF_LEN - hdma_usart1_rx->Instance->NDTR;//获取接收数据长度
    hdma_usart1_rx->Instance->NDTR = UART1_RX_BUF_LEN;//重新设定数据长度
    uint32_t target_index = (hdma_usart1_rx->Instance->CR & DMA_SxCR_CT) ? 1 : 0;//获取当前缓冲区
    hdma_usart1_rx->Instance->CR ^= DMA_SxCR_CT; //反转缓冲区
    __HAL_DMA_ENABLE(hdma_usart1_rx);//使能DMA

    // 更鲁棒的数据帧处理：允许长度在一定范围内
    if(this_time_rx_len >= Device::VT03::VT03_FRAME_LENGTH - 2 && 
       this_time_rx_len <= Device::VT03::VT03_FRAME_LENGTH + 2)
    {
        Device::VT03 &hVT03 = Device::VT03::getInstance();
        hVT03.receiveMessage(UART1_DMA_BUF[target_index]);//解码数据
        success_count++;
        error_count = 0; // 成功接收后重置错误计数
    }
    else if (this_time_rx_len > 0)
    {
        // 长度不正确，可能是分段上电导致的同步问题
        error_count++;
        if (error_count > 50) // 连续50次错误，可能需要重新初始化
        {
            // 这里可以添加重新初始化的逻辑
            error_count = 0;
        }
    }


}

extern "C" void USART3_IDEL_IRQHandler(UART_HandleTypeDef *huart){

    //__HAL_UART_CLEAR_PEFLAG(&huart3);

    static uint16_t this_time_rx_len = 0;
    DMA_HandleTypeDef* hdma_usart3_rx = huart->hdmarx;

    __HAL_DMA_DISABLE(hdma_usart3_rx); //失能DMA
    this_time_rx_len = UART3_RX_BUF_LEN - hdma_usart3_rx->Instance->NDTR;//获取接收数据长度
    hdma_usart3_rx->Instance->NDTR = UART3_RX_BUF_LEN;//重新设定数据长度
    uint32_t target_index = (hdma_usart3_rx->Instance->CR & DMA_SxCR_CT) ? 1 : 0;//获取当前缓冲区
    hdma_usart3_rx->Instance->CR ^= DMA_SxCR_CT; //反转缓冲区
    __HAL_DMA_ENABLE(hdma_usart3_rx);//使能DMA

    if(this_time_rx_len == Device::DBus::DBUS_FRAME_LENGTH)
    {
        Device::DBus &hdbus = Device::DBus::getInstance();
        hdbus.receiveMessage(UART3_DMA_BUF[target_index]);//解码数据
    }

}


//static uint8_t test_data[] = {
//        0xa5, 0x10, 0x00, 0x02, 0x8f, 0x2, 0x2, 0x83, 0x5d, 0x96, 0x00,
//        0x53, 0x8d, 0x6f, 0x40, 0x3c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
//        0xc9, 0x4a, 0x00, 0x00
//};

extern "C" void USART6_IDEL_IRQHandler(UART_HandleTypeDef *huart){

    static uint16_t this_time_rx_len = 0;
    DMA_HandleTypeDef* hdma_usart6_rx = huart->hdmarx;

    __HAL_DMA_DISABLE(hdma_usart6_rx); //失能DMA
    this_time_rx_len = UART6_RX_BUF_LEN - hdma_usart6_rx->Instance->NDTR;//获取接收数据长度
    hdma_usart6_rx->Instance->NDTR = UART6_RX_BUF_LEN;//重新设定数据长度
    uint32_t target_index = (hdma_usart6_rx->Instance->CR & DMA_SxCR_CT) ? 1 : 0;//获取当前缓冲区
    hdma_usart6_rx->Instance->CR ^= DMA_SxCR_CT; //反转缓冲区
    __HAL_DMA_ENABLE(hdma_usart6_rx);//使能DMA

    if(this_time_rx_len >= Device::Referee::MIN_FRAME_SIZE)
    {
        Device::Referee &href = Device::Referee::getInstance();
        href.pushData(UART6_DMA_BUF[target_index], this_time_rx_len);//存入FIFO
        //test_data[3] = HAL_GetTick()%255;
        //href.pushData(test_data, 25);
    }

}

