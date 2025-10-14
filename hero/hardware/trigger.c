#include "trigger.h"
#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim1;

//PE9 TIM1_CH1、PE11 TIM1_CH2
void trigger_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_OC_InitTypeDef sConfigOC = {0};

    // 使能GPIOE时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // 配置GPIO为定时器复用功能
    GPIO_InitStructure.Pin = GPIO_PIN_9 | GPIO_PIN_11;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);

    // 在HAL库中，定时器基本初始化已经在CubeMX生成的代码中完成
    // 这里只需要更新定时器参数和配置PWM通道

    // 更新定时器预分频值和自动重装载值
    htim1.Instance->PSC = 168 - 1;  // 168MHz
    htim1.Instance->ARR = 20000 - 1;

    // 配置PWM通道1
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1000;  // 初始占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    // 配置PWM通道2，参数与通道1相同
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // 使能TIM1的主输出
    __HAL_TIM_MOE_ENABLE(&htim1);

    // 初始设置摩擦轮为关闭状态
    fric_off();
}

void fric_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, FRIC_OFF);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, FRIC_OFF);
}

void fric1_on(uint16_t cmd)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmd);
}

void fric2_on(uint16_t cmd)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmd);
}
