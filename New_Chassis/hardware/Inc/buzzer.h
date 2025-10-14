#ifndef BUZZER_H
#define BUZZER_H
#include "main.h"
#include "stm32f4xx_hal.h"

// 需要在主程序定义 extern TIM_HandleTypeDef htim4;

void buzzer_init(uint16_t arr, uint16_t psc);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);

#endif
