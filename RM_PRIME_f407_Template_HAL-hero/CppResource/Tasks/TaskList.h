#ifndef F407_RM_TMPLATE_HAL_TASKLIST_H
#define F407_RM_TMPLATE_HAL_TASKLIST_H

#include "main_cpp.h"
#include "Debug.h"

#ifdef __cplusplus
extern "C" {
#endif


[[noreturn]] void TestTask(void const * argument);
[[noreturn]] void DebugTask(void const * argument);
[[noreturn]] void INS_Task(void const * argument);
[[noreturn]] void MotionControlTask(void const * argument);
[[noreturn]] void RefereeProcessTask(void const * argument);
[[noreturn]] void ShootTask(void const * argument);
[[noreturn]] void NanoTask(void const * argument);
[[noreturn]] void ClientUI_Task(void const * argument);

#ifdef __cplusplus
}
#endif



#endif //F407_RM_TMPLATE_HAL_TASKLIST_H
