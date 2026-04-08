#include "TaskList.h"
#include "Referee.h"

using namespace Device;

[[noreturn]] void RefereeProcessTask(void const * argument){

    Referee &href = Referee::getInstance();

    while (true){
        href.ProcessData();

        osDelay(2);
    }
}