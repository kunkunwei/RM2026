#include "temperature.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim10;

//PF6 TIM10_CH1
void temp_pwm_init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 使能GPIOF时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // 配置GPIO为定时器复用功能
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF3_TIM10;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);

    // 在HAL库中，定时器基本初始化已经在CubeMX生成的代码中完成
    // 这里只需要配置PWM通道和启动定时器

    // 更新定时器预分频值和自动重装载值
    htim10.Instance->PSC = psc - 1;
    htim10.Instance->ARR = arr - 1;

    // 配置PWM通道
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1);

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void bmi_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm);
}
