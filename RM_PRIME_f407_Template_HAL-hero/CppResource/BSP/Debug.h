#ifndef F407_RM_TMPLATE_HAL_DEBUG_H
#define F407_RM_TMPLATE_HAL_DEBUG_H

#include "main_cpp.h"
#include <type_traits>

#define hdebuguart huart1

class Debug {
public:

    static void print(const char* fmt, ...);
    static void print_vofa(float* pList, int len);

    template<typename T>
    typename std::enable_if<std::is_integral<T>::value, void>::type
    static print_vofa(T* pList, int len){
        for (int i = 0; i < len; ++i) {
            float fv = static_cast<float>(pList[i]);
            HAL_UART_Transmit(&hdebuguart, (uint8_t *)&fv, sizeof(float), 0xFFFF);
        }
        uint8_t tail[4]{0x00, 0x00, 0x80, 0x7f};
        HAL_UART_Transmit(&hdebuguart, tail, 4, 0xFFFF);

    }
};


#endif
