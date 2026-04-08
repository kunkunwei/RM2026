#include "main_cpp.h"
#include "Interface_UART.h"
#include "InterfaceCAN.h"

#include "IST8310.h"
#include "BMI088.h"


void app_init_function(void) {

    HAL_Delay(200);
    BSP::CAN_UserInit();
    BSP::UART_UserInit();

    Device::BMI088 &hIMU = Device::BMI088::getInstance();
    auto& hIST = Device::IST8310::getInstance();
    hIMU.Init();
    hIST.Init();

}

