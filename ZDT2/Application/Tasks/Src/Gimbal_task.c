#include "Gimbal_task.h"

#include <stdio.h>

#include "main.h"
#include "stepper_can.h"
#include "stepper_motor.h"
#include "cmsis_os.h"
#include "remote_control.h"

#define YAW_ANGLE_MIN   -180.0f
#define YAW_ANGLE_MAX    180.0f
#define PITCH_ANGLE_MIN   -45.0f
#define PITCH_ANGLE_MAX    45.0f
#define CONTROL_PERIOD_MS 100          // 控制周期100ms
#define DEFAULT_SPEED 300              // 默认速度200 RPM
#define DEFAULT_ACCELERATION 0x01      // 默认加速度档位

// 全局目标角度
float target_yaw_angle = 0;
float target_pitch_angle = 0;

// 引用上位机数据
extern PC_Data_Typedef pc_data;
extern osThreadId StartUserTaskHandle;

void Gimbal_Task(void const * argument)
{
    osDelay(1000);

    // 初始化电机
    StepperCAN_Enable(&yaw_motor_Info, true);
    osDelay(200);
    StepperCAN_Enable(&pitch_motor_Info, true);
    osDelay(200);

    TickType_t systick = 0;
    TickType_t last_pc_send_tick = 0;

    for(;;)
    {
        systick = osKernelSysTick();

        // 直接使用上位机发送的角度
        target_yaw_angle = pc_data.yaw_angle;
        target_pitch_angle = pc_data.pitch_angle;

        // 限制角度范围
        if (target_yaw_angle > YAW_ANGLE_MAX) target_yaw_angle = YAW_ANGLE_MAX;
        if (target_yaw_angle < YAW_ANGLE_MIN) target_yaw_angle = YAW_ANGLE_MIN;
        if (target_pitch_angle > PITCH_ANGLE_MAX) target_pitch_angle = PITCH_ANGLE_MAX;
        if (target_pitch_angle < PITCH_ANGLE_MIN) target_pitch_angle = PITCH_ANGLE_MIN;

        // 直接控制电机到目标位置
        StepperCAN_SetAbsolutePosition(&yaw_motor_Info, target_yaw_angle, DEFAULT_SPEED, DEFAULT_ACCELERATION);
        osDelay(10);
        StepperCAN_SetAbsolutePosition(&pitch_motor_Info, target_pitch_angle, DEFAULT_SPEED, DEFAULT_ACCELERATION);

        // 定期发送电机状态给上位机
        TickType_t now = osKernelSysTick();
        if (now - last_pc_send_tick >= pdMS_TO_TICKS(100)) {
            PC_Send_Motor_Angles(yaw_motor_Info.current_angle,
                               pitch_motor_Info.current_angle, 0x01);
            last_pc_send_tick = now;
        }

        // 通知User_Task处理位置读取
        if (StartUserTaskHandle) {
            xTaskNotifyGive(StartUserTaskHandle);
        }

        osDelayUntil(&systick, CONTROL_PERIOD_MS);
    }
}
