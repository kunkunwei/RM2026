#include "uart1.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

/*------------------printf函数重定向-------------------*/
#include <stdio.h> 
struct __FILE 
{ 
	int handle; 

}; 
FILE __stdout;       
   
void _sys_exit(int x) 
{ 
	x = x; 
} 

int fputc(int ch, FILE *f)
{      
	// 使用HAL库的发送函数
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}
/*-----------------------------------------------------*/

//UART1_Tx PA9    UART1_Rx PB7
void UART1_Init(void)
{
    // 在HAL库中，UART和GPIO的初始化已经在CubeMX生成的代码中完成
    // 这里只需要启动UART接收中断（如果需要）

    // 配置UART1接收中断
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART_RX_BUF[0], 1);
}

// 添加HAL库的UART接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 判断接收到的数据是否为帧尾
        if(USART_RX_STA & 0x8000)  // 接收已完成
        {
            // 重新开始接收
            USART_RX_STA = 0;
        }
        else if(USART_RX_BUF[USART_RX_STA & 0X3FFF] == 0x0A)  // 判断是否收到换行符
        {
            if((USART_RX_STA & 0X3FFF) > 0 && USART_RX_BUF[(USART_RX_STA & 0X3FFF) - 1] == 0x0D)  // 判断前一个字符是否为回车符
            {
                USART_RX_STA |= 0x8000;  // 接收完成
            }
            else
            {
                USART_RX_BUF[USART_RX_STA & 0X3FFF] = 0x0A;
                USART_RX_STA++;
            }
        }
        else
        {
            USART_RX_BUF[USART_RX_STA & 0X3FFF] = huart->Instance->DR & 0xFF;
            USART_RX_STA++;

            if(USART_RX_STA > (USART_REC_LEN - 1))
            {
                USART_RX_STA = 0;  // 溢出，重新开始接收
            }
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&USART_RX_BUF[USART_RX_STA & 0X3FFF], 1);
    }
}
