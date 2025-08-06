//
// Created by kun on 25-7-13.
//

#include "../Inc/user_task.h"
#include "cmsis_os.h"
#include "main.h"
// #include "stepper_pwm_control.h"
#include "usart.h"
// #include "vofa.h"
//
// typedef struct {
//     int32_t last_pos;
//     uint32_t last_tick;
// } MotorVelTracker;
// MotorVelTracker vel_tracker = {
//     .last_pos = 0,
//     .last_tick = 0
// };
void User_Task(void const * argument)
{
    /* USER CODE BEGIN User_Task */
    // osKernelSysTick单位是ms
    // TickType_t systick = 0;
    /* Infinite loop */
    for(;;)
    {
        // User Task Code Here
        // Stepper_Update();
        // get_emm42_motor_measure(stepper_motor,0);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
        // Vofa_SendRCChannels(&huart6,get_emm42_motor_measure_point(0));
        osDelay(2); // Delay for 1 second
    }
    /* USER CODE END User_Task */
}