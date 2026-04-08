#ifndef __CAN_TASK__
#define __CAN_TASK__
#include <stdint.h>

#include "bsp_can.h"

void Can_Task(void const *argument);

void Damiao_Motor_Enable();
#endif // !__CAN_TASK__
