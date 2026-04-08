#ifndef F407_RM_TMPLATE_HAL_INTERFACE_UART_H
#define F407_RM_TMPLATE_HAL_INTERFACE_UART_H

#include "main_cpp.h"

#define UART_TIME_OUT_COUNT 1000

#define UART3_RX_BUF_LEN 36
#define UART6_RX_BUF_LEN 256
#define UART1_RX_BUF_LEN 42

namespace BSP{
    void UART_UserInit();
}

#endif
