#include "Debug.h"
#include "cstdarg"

extern "C" int __io_putchar(int ch){
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}


void Debug::print(const char* fmt, ...){
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void Debug::print_vofa(float* pList, int len){
    uint8_t tail[4]{0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&hdebuguart, (uint8_t *)pList, sizeof(float)*len, 0xFFFF);
    HAL_UART_Transmit(&hdebuguart, tail, 4, 0xFFFF);
}

