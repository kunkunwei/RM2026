#include "TaskList.h"
#include "Buzzer.h"
#include "INS_Device.h"
#include "NanoMsg.h"
#include "usbd_cdc_if.h"
#include "Debug.h"

using namespace Device;

static uint8_t rx_buf[256];

[[noreturn]] void NanoTask(void const * argument){

    auto &hINS = INS_Device::getInstance();
    auto &hnano = NanoMsg::getInstance();

    while (1){

        auto angle = hINS.getAngle();
        hnano.sendMsg(angle.yaw, angle.pitch);
        //hnano.sendMsg();

        uint32_t len = CDC_Available_FS();
        if(len > 0){
            uint32_t read_len = CDC_Read_FS(rx_buf, len);
            hnano.pushData(rx_buf, read_len);
        }

        hnano.ProcessData();


        osDelay(10);
    }

}




//[[noreturn]] void DebugTask(void const * argument){
//
//    float pack[5];
//    auto &hnano = NanoMsg::getInstance();
//
//    while (true){
//
//        auto angle = hnano.getControlCmd();
//        pack[0] = angle.yaw;
//        pack[1] = angle.pitch;
////        pack[2] = hINS.getAccel().x;
////        pack[3] = hINS.getAccel().y;
//
//        Debug::print_vofa(pack, 2);
//
//        osDelay(50);
//    }
//}