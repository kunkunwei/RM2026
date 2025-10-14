#include "buzzer.h"
#include "stm32f4xx_hal.h"

// 定义一个定时器句柄
extern TIM_HandleTypeDef htim4; // 需在 main.c 或相关文件定义并初始化

void buzzer_init(uint16_t arr, uint16_t psc)
{
    // 配置定时器参数
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = psc;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = arr;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim4);

    // 配置PWM通道
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3);

    buzzer_off();
}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    // 可选：重新设置分频
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void buzzer_off(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
}
