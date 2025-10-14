#ifndef UI_TASK_H
#define UI_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
extern void UI_Task(void *pvParameters);

extern uint8_t shoot_vel_state;
extern uint8_t auto_state;
extern uint8_t roation_state;

#endif