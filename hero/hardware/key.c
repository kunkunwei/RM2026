#include "key.h"
#include "led.h"
#include "stm32f4xx_hal.h"

void key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 在HAL库中，GPIO和EXTI的初始化大部分已经在CubeMX生成的代码中处理
    // 这里我们可以添加额外的配置

    // 启用GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 配置PA0为输入上拉，因为是按键
    GPIO_InitStructure.Pin = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;  // 配置为下降沿触发中断
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置NVIC，设置中断优先级
    HAL_NVIC_SetPriority(EXTI0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

// HAL库中，外部中断处理函数名称有所变化
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)
    {
        led_blue_toggle();
    }
}
