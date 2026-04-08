#ifndef F407_RM_TMPLATE_HAL_RGB_LED_H
#define F407_RM_TMPLATE_HAL_RGB_LED_H

#include "main_cpp.h"

namespace Device
{
    class RGB_LED
    {
    public:
        void setColor(uint32_t aRGB);
        uint32_t getColor();
        void setColorPhase(float deg);

        //构造函数调用了HAL相关的函数，请不要在HAL初始化之前获取实例
        //建议在main函数中调用BSP_Init()之后获取实例
        static RGB_LED &getInstance();

    private:
        uint32_t color;

        RGB_LED();
        ~RGB_LED() = default;
        RGB_LED(const RGB_LED&) = delete;
        RGB_LED& operator=(const RGB_LED&) = delete;

    };
}

#endif
