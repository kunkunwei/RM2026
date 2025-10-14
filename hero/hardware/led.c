#include "led.h"
#include "stm32f4xx_hal.h"

void led_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    // 启用GPIOH时钟
    __HAL_RCC_GPIOH_CLK_ENABLE();

    // 配置GPIO引脚
    GPIO_InitStructure.Pin = GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

    // led_blue_on();
    led_green_on();
    // led_red_on();
}

void led_blue_on(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
}

void led_blue_off(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
}

void led_blue_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
}

void led_green_on(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
}

void led_green_off(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
}

void led_green_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
}

void led_red_on(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
}

void led_red_off(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
}

void led_red_toggle(void)
{
    HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
}
