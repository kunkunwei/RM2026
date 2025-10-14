#include "stm32f4xx_hal.h"
#include "relays.h"
#include "remote_control.h"

const RC_ctrl_t *local_rc;

void Relays_On(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
}

void Relays_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
}

void Relays_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOC时钟
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置GPIO
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 默认打开继电器
    Relays_On();

    // 获取遥控器指针
    local_rc = get_remote_control_point();
}

void Relays_Judge(void)
{
    if(local_rc->key.v & KEY_PRESSED_OFFSET_Z)
    {
        Relays_On();
    }
    if(local_rc->key.v & KEY_PRESSED_OFFSET_G)
    {
        Relays_Off();
    }
}
