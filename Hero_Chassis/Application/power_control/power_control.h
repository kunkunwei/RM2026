#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "Chassis_Task.h"

typedef struct 
{
    float current_power;
    float cap_percent;
}cap_measure_t;

void chassis_power_control(chassis_move_t *chassis_power_control);
#endif // !1