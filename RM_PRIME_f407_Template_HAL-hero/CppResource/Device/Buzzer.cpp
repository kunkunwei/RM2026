#include "Buzzer.h"

using namespace Device;

Buzzer::Buzzer():state(false) {

    HAL_TIM_Base_Start(&htim4);
    __HAL_TIM_PRESCALER(&htim4, 169);
    setFrequency(440);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);

}

Buzzer &Buzzer::getInstance() {
    static Buzzer instance;
    return instance;
}

void Buzzer::on() {

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    state = true;
}

void Buzzer::off() {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    state = false;
}

void Buzzer::toggle()
{
    if (state)
        off();
    else
        on();
}

bool Buzzer::isOn() {
    return state;
}

void Buzzer::setFrequency(uint32_t f){
    uint32_t counter = 1000000 / f;
    if(counter > 65535)
        __HAL_TIM_SetAutoreload(&htim4, 65535);
    else
        __HAL_TIM_SetAutoreload(&htim4, counter);

    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, (__HAL_TIM_GetAutoreload(&htim4) + 1) / 2);
}