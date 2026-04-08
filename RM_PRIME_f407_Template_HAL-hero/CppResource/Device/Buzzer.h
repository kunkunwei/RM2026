#ifndef BUZZER_H
#define BUZZER_H

#include "main_cpp.h"

namespace Device {
    class Buzzer {
    public:
        void on();
        void off();
        void toggle();
        bool isOn();

        //单位Hz
        static void setFrequency(uint32_t f);

        //构造函数调用了HAL相关的函数，请不要在HAL初始化之前获取实例
        //建议在main函数中调用BSP_Init()之后获取实例
        static Buzzer &getInstance();

        static constexpr uint32_t ToneList[] = {
                40000,    //Nop
                440,  //A4
                494,  //B4
                523,  //C5
                587,  //D5
                659,  //E5
                698,  //F5
                784,  //G5
                880,  //A5
                988,  //B5
                1046, //C6
                1174, //D6
                1318, //E6
                1397, //F6
                1568, //G6
                1760, //A6
                1976, //B6
                2093, //C7
                2349, //D7
                2637, //E7
                2794, //F7
                3136, //G7
                3520, //A7
        };

    private:
        bool state;

        Buzzer();
        ~Buzzer() = default;
        Buzzer(const Buzzer&) = delete;
        Buzzer& operator=(const Buzzer&) = delete;

    };
}


#endif //BUZZER_H
