#include "Gimbal_task.h"
#include "main.h"
#include "stepper_can.h"
#include "stepper_motor.h"
#include "cmsis_os.h"
#include "remote_control.h"

#define YAW_ANGLE_MIN   -180.0f
#define YAW_ANGLE_MAX    180.0f
#define PITCH_ANGLE_MIN   -45.0f
#define PITCH_ANGLE_MAX    45.0f
#define RC_CH_MAX        660.0f
#define RC_CH_MIN       -660.0f
#define RC_DEADZONE      15
#define ANGLE_RATE       2.0f         // 角度变化速率 度/秒
#define CONTROL_PERIOD_MS 20          // 控制周期，单位：ms

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
    osDelay(100);
    StepperCAN_Enable(&pitch_motor_Info, true);
    osDelay(100);

    TickType_t systick = 0;
    TickType_t last_pc_send_tick = 0;

    for(;;)
    {
        systick = osKernelSysTick();

        // 检查上位机数据是否有效且未超时
        uint32_t current_time = HAL_GetTick();
        bool pc_data_valid = pc_data.data_valid && (current_time - pc_data.last_update) < 500;

        if(pc_data_valid) {
            // 使用上位机设定的目标角度
            target_yaw_angle = pc_data.yaw_angle;
            target_pitch_angle = pc_data.pitch_angle;

            // 限制角度范围
            if (target_yaw_angle > YAW_ANGLE_MAX) target_yaw_angle = YAW_ANGLE_MAX;
            if (target_yaw_angle < YAW_ANGLE_MIN) target_yaw_angle = YAW_ANGLE_MIN;
            if (target_pitch_angle > PITCH_ANGLE_MAX) target_pitch_angle = PITCH_ANGLE_MAX;
            if (target_pitch_angle < PITCH_ANGLE_MIN) target_pitch_angle = PITCH_ANGLE_MIN;
        } else {
            // 使用遥控器数据进行手动控制
            int16_t rc_yaw_raw = remote_ctrl.rc.ch[0];
            int16_t rc_pitch_raw = remote_ctrl.rc.ch[1];

            // 死区处理
            if (abs(rc_yaw_raw) < RC_DEADZONE) rc_yaw_raw = 0;
            if (abs(rc_pitch_raw) < RC_DEADZONE) rc_pitch_raw = 0;

            // 增量式控制：根据遥控器输入调整目标角度
            float yaw_delta = ((float)rc_yaw_raw / RC_CH_MAX) * ANGLE_RATE * (CONTROL_PERIOD_MS / 1000.0f);
            float pitch_delta = ((float)rc_pitch_raw / RC_CH_MAX) * ANGLE_RATE * (CONTROL_PERIOD_MS / 1000.0f);

            target_yaw_angle += yaw_delta;
            target_pitch_angle += pitch_delta;

            // 限制角度范围
            if (target_yaw_angle > YAW_ANGLE_MAX) target_yaw_angle = YAW_ANGLE_MAX;
            if (target_yaw_angle < YAW_ANGLE_MIN) target_yaw_angle = YAW_ANGLE_MIN;
            if (target_pitch_angle > PITCH_ANGLE_MAX) target_pitch_angle = PITCH_ANGLE_MAX;
            if (target_pitch_angle < PITCH_ANGLE_MIN) target_pitch_angle = PITCH_ANGLE_MIN;
        }

        // 定期发送电机状态给上位机
        TickType_t now = osKernelSysTick();
        if (now - last_pc_send_tick >= pdMS_TO_TICKS(100)) { // 100ms发送一次
            if (yaw_motor_Info.position_valid && pitch_motor_Info.position_valid) {
                PC_Send_Motor_Angles(yaw_motor_Info.current_angle,
                                   pitch_motor_Info.current_angle, 0x01);
            }
            last_pc_send_tick = now;
        }

        // 修改：确保持续通知User_Task处理电机控制
        if (StartUserTaskHandle) {
            xTaskNotifyGive(StartUserTaskHandle);
        }

        osDelayUntil(&systick, CONTROL_PERIOD_MS);
    }
}
