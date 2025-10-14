#include "led.h"

void led_Init(void)
{


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
