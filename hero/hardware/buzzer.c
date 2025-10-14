#include "buzzer.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim4;

//PD14 TIM4 OC3
//定时器时钟为84MHz，分频系数psc为84，计数频率就为1MHZ，arr为重装载值
void buzzer_init(uint16_t arr, uint16_t psc)
{
    // 在HAL库中，TIM的初始化已经在CubeMX生成的tim.c中实现
    // 这里我们只需要配置PWM通道

    TIM_OC_InitTypeDef sConfigOC = {0};

    // 更新预分频值和自动重装载值
    htim4.Instance->PSC = psc - 1;
    htim4.Instance->ARR = arr - 1;

    // 配置PWM通道
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;  // 初始占空比为0
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    // 配置TIM4的通道3(PD14)
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // 默认关闭蜂鸣器
    buzzer_off();
}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    // 更新预分频值
    htim4.Instance->PSC = psc;

    // 设置PWM占空比
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
}

void buzzer_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}
