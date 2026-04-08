#include "RGB_LED.h"

namespace Device{
    RGB_LED::RGB_LED()
    {
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

        setColor(0x00000000);
    }

    void RGB_LED::setColor(uint32_t aRGB){
        uint8_t alpha;
        uint16_t red,green,blue;

        alpha = (aRGB & 0xFF000000) >> 24;
        red = ((aRGB & 0x00FF0000) >> 16) * alpha;
        green = ((aRGB & 0x0000FF00) >> 8) * alpha;
        blue = ((aRGB & 0x000000FF) >> 0) * alpha;

        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, blue);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, green);
        __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, red);

        color = aRGB;
    }

    void RGB_LED::setColorPhase(float deg){

        float r = 0.f, g = 0.f, b = 0.f;

        if(deg < 60.f){
            r = 1.f;
            g = deg / 60.f;
            b = 0.f;
        }
        else if(deg < 120.f){
            r = (120.f - deg) / 60.f;
            g = 1.f;
            b = 0.f;
        }
        else if(deg < 180.f){
            r = 0.f;
            g = 1.f;
            b = (deg - 120.f) / 60.f;
        }
        else if(deg < 240.f){
            r = 0.f;
            g = (240.f - deg) / 60.f;
            b = 1.f;
        }
        else if(deg < 300.f){
            r = (deg - 240.f) / 60.f;
            g = 0.f;
            b = 1.f;
        }
        else{
            r = 1.f;
            g = 0.f;
            b = (360.f - deg) / 60.f;
        }

        setColor(0xFF000000 | ((uint8_t)(r * 255) << 16) | ((uint8_t)(g * 255) << 8) | ((uint8_t)(b * 255) << 0));
    }

    uint32_t RGB_LED::getColor() {
        return color;
    }

    RGB_LED& RGB_LED::getInstance() {
        static RGB_LED instance;
        return instance;
    }
}