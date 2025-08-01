#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
#include "stepper_can.h"
#include "stepper_motor.h"
// #include "stepper_pwm_control.h"
#include "tim.h"
#include "usart.h"

//
#define rc_deadline_limit(input, output, dealine)        \
{                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
}


void Gimbal_Task(void const * argument)
{
  /* Infinite loop */

    //一个周期是1us
    //100%占空比是1000


    osDelay(800);
    // osDelay(800);

    StepperCAN_Enable( &yaw_motor_Info,ENABLE);
    StepperCAN_Enable( &pitch_motor_Info,ENABLE);
    TickType_t systick = 0;
    // StepperCAN_SetSpeed(&yaw_motor_Info, 200,  100);

    for(;;)
    {
        // Stepper_Update();
        systick = osKernelSysTick();
        StepperCAN_SetPositionAngle(&yaw_motor_Info,50, 20, 10, 1);
        StepperCAN_SetPositionAngle(&pitch_motor_Info,50, 20, 10, 1);
        // StepperCAN_ReadSystemStatus(CAN1_YAW_MOTOR_ID);
        osDelayUntil(&systick,2);
    }
}
