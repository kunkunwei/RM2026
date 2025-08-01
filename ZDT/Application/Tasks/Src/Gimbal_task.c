#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
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
#define RC_DEADZONE      10
#define ANGLE_SEND_THRESHOLD 0.2f  // 角度变化阈值，单位：度
#define CONTROL_PERIOD_MS 5        // 控制周期，单位：ms
#define PC_SEND_PERIOD_MS 100      // 上位机发送周期，单位：ms

// 全局目标角度，供User_Task等模块引用
float target_yaw_angle = 0;
float target_pitch_angle = 0;

// 引用 stepper_response.c 的全局变量
extern int32_t g_position_yaw, g_position_pitch;
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

// 引用上位机数据
extern PC_Data_Typedef pc_data;

// 角度转换函数
static float position_to_angle(int32_t position) {
    return (float)position * 360.0f / 65536.0f;
}

void Gimbal_Task(void const * argument)
{
    osDelay(1000);
    StepperCAN_Enable(&yaw_motor_Info, true);
    StepperCAN_Enable(&pitch_motor_Info, true);

    TickType_t systick = 0;
    TickType_t last_pc_send_tick = 0;

    for(;;)
    {
        systick = osKernelSysTick();

        // 使用统一的角度转换公式
        float yaw_actual_angle = position_to_angle(g_position_yaw);
        float pitch_actual_angle = position_to_angle(g_position_pitch);

        // 检查是否有有效的上位机数据
        uint32_t current_time = HAL_GetTick();
        bool pc_data_timeout = (current_time - pc_data.last_update) > 500;

        if(pc_data.data_valid && !pc_data_timeout) {
            // 使用上位机数据作为目标角度
            target_yaw_angle = pc_data.yaw_angle;
            target_pitch_angle = pc_data.pitch_angle;
        } else {
            // 使用遥控器数据
            int16_t rc_yaw_raw = remote_ctrl.rc.ch[0];
            int16_t rc_pitch_raw = remote_ctrl.rc.ch[1];

            // 添加死区处理
            if (abs(rc_yaw_raw) < RC_DEADZONE) rc_yaw_raw = 0;
            if (abs(rc_pitch_raw) < RC_DEADZONE) rc_pitch_raw = 0;

            // 映射到角度范围并限制
            target_yaw_angle = ((float)rc_yaw_raw) * (YAW_ANGLE_MAX - YAW_ANGLE_MIN) / (RC_CH_MAX - RC_CH_MIN);
            target_pitch_angle = ((float)rc_pitch_raw) * (PITCH_ANGLE_MAX - PITCH_ANGLE_MIN) / (RC_CH_MAX - RC_CH_MIN);

            // 限制角度范围
            if (target_yaw_angle > YAW_ANGLE_MAX) target_yaw_angle = YAW_ANGLE_MAX;
            if (target_yaw_angle < YAW_ANGLE_MIN) target_yaw_angle = YAW_ANGLE_MIN;
            if (target_pitch_angle > PITCH_ANGLE_MAX) target_pitch_angle = PITCH_ANGLE_MAX;
            if (target_pitch_angle < PITCH_ANGLE_MIN) target_pitch_angle = PITCH_ANGLE_MIN;
        }

        TickType_t now = osKernelSysTick();

        // 定期发送电机实时角度值给上位机
        if (now - last_pc_send_tick >= PC_SEND_PERIOD_MS) {
            PC_Send_Motor_Angles(yaw_actual_angle, pitch_actual_angle, 0x01);
            last_pc_send_tick = now;
        }

        // 移除所有电机控制和死机检测逻辑，由User_Task专门负责

        osDelayUntil(&systick, CONTROL_PERIOD_MS);
    }
    // 永远不会到这里，防止意外退出
    while(1) { osDelay(1000); }
}

// 串口接收相关代码已移除，CAN反馈处理请在 stepper_response.c/h 实现
