//
// Created by kun on 25-7-13.
//

#include "../Inc/user_task.h"
#include "cmsis_os.h"
#include "main.h"
// #include "stepper_pwm_control.h"
#include "stepper_can.h"
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
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;
extern osThreadId StartUserTaskHandle;

// 供 stepper_response.c 使用的全局变量
uint8_t yaw_wait_angle_done = 0;
uint8_t pitch_wait_angle_done = 0;

void User_Task(void const * argument)
{
    uint8_t motor_priority = 0; // 0: YAW优先，1: PITCH优先
    static uint8_t last_sent = 0; // 0: 未发，1: 已发但未收到反馈
    for(;;)
    {
        // 等待定时器中断通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 只交替单发，且必须等上一次反馈释放信号量后才允许下次发送
        if (motor_priority == 0) {
            // 如果有角度控制指令未完成，则不发送读取位置指令
            if (yaw_wait_angle_done == 0 && osSemaphoreGetCount(can_cmd_sem_yawHandle) > 0 && last_sent == 0) {
                StepperCAN_ReadPosition(CAN1_YAW_MOTOR_ID, NULL);
                last_sent = 1;
            } else if (osSemaphoreGetCount(can_cmd_sem_yawHandle) > 0 && last_sent == 1) {
                // 上一次反馈已到，切换到下一个电机
                motor_priority = 1;
                last_sent = 0;
            }
        } else {
            if (pitch_wait_angle_done == 0 && osSemaphoreGetCount(can_cmd_sem_pitchHandle) > 0 && last_sent == 0) {
                StepperCAN_ReadPosition(CAN1_PITCH_MOTOR_ID, NULL);
                last_sent = 1;
            } else if (osSemaphoreGetCount(can_cmd_sem_pitchHandle) > 0 && last_sent == 1) {
                // 上一次反馈已到，切换到下一个电机
                motor_priority = 0;
                last_sent = 0;
            }
        }

        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    }
    while(1) { osDelay(1000); }
}
