#include "TaskList.h"
#include "Debug.h"
#include "BMI088.h"
#include "IST8310.h"
#include "INS_Device.h"


using namespace Device;
using namespace Component;
//using namespace BSP;

static TickType_t INS_LastWakeTime;


BMI088::Measurement tmp;

[[noreturn]] void INS_Task(void const * argument){

    INS_LastWakeTime = xTaskGetTickCount();
    INS_Device::Vector3 acc_raw{}, gyro_raw{}, meg_raw{{{0.f ,0.f,0.f}}};
    float temperature = 0.f;
    float dt = 0.002f;

    BMI088 &hIMU = BMI088::getInstance();
    IST8310 &hIST = IST8310::getInstance();

    INS_Device &hINS = INS_Device::getInstance();
    hINS.Init();



    while (true){

        hIMU.Update();
        hIST.Update();

        hIMU.GetMeasurement(acc_raw.data, gyro_raw.data, &temperature);
        hIST.GetMeasurement(meg_raw.data);

        hINS.Insert(acc_raw, gyro_raw, meg_raw, dt);

        vTaskDelayUntil(&INS_LastWakeTime, 2);
    }
}


//void DebugTask(void const * argument){
//
//    INS_Device &hINS = INS_Device::getInstance();
//
//    while (1){
//
//        Debug::print_vofa(hINS.getAngle().data, 3);
//        //Debug::print_vofa(tmp.acc, 3);
//
//        osDelay(50);
//    }
//}