//
// Created by kun on 25-7-13.
//

#include "../Inc/user_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "stepper_can.h"
#include "usart.h"
#include "math.h"

extern osThreadId StartUserTaskHandle;
extern float target_yaw_angle, target_pitch_angle;
extern int32_t g_position_yaw, g_position_pitch;

#define POSITION_READ_PERIOD_MS 50   // 位置读取周期50ms（20Hz）
#define CONTROL_CHECK_PERIOD_MS 20   // 控制检查周期20ms
#define ANGLE_THRESHOLD 0.3f         // 角度控制阈值
#define MAX_CONTROL_SPEED 0x200      // 最大控制速度
#define ACCELERATION 0x40            // 加速度参数
#define POSITION_TOLERANCE 1.0f      // 位置到位容差（度）

typedef struct {
    StepperMotor_Info_t* motor;
    float* target_angle;
    TickType_t last_read_tick;
    TickType_t last_control_tick;
    bool position_reading;
    bool controlling;
    float last_target_angle;         // 上次的目标角度
    TickType_t motion_start_tick;    // 运动开始时间
} MotorController_t;

// 角度转换函数
static float position_to_angle(int32_t position) {
    return (float)position * 360.0f / 65536.0f;
}

// 角度差值计算（考虑跨越±180°边界）
static float angle_difference(float target, float current) {
    float diff = target - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

// 检查电机是否到达目标位置
static bool is_motor_in_position(MotorController_t* ctrl) {
    if (!ctrl->motor->position_valid) return false;

    float angle_diff = angle_difference(*(ctrl->target_angle), ctrl->motor->current_angle);
    return fabsf(angle_diff) <= POSITION_TOLERANCE;
}

void User_Task(void const * argument)
{
    // 等待系统初始化完成
    osDelay(2000);

    // 初始化电机控制器结构
    MotorController_t controllers[2] = {
        {&yaw_motor_Info, &target_yaw_angle, 0, 0, false, false, 0.0f, 0},
        {&pitch_motor_Info, &target_pitch_angle, 0, 0, false, false, 0.0f, 0}
    };

    uint8_t active_motor = 0; // 当前处理的电机索引
    TickType_t last_task_run = osKernelSysTick();

    for(;;)
    {
        // 修改：使用超时等待，确保任务能够持续运行
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50)); // 50ms超时

        TickType_t now = osKernelSysTick();

        // 确保任务至少每50ms运行一次
        if ((now - last_task_run) < pdMS_TO_TICKS(20)) {
            osDelay(5);
            continue;
        }
        last_task_run = now;

        MotorController_t* ctrl = &controllers[active_motor];

        // 检查目标角度是否发生变化
        bool target_changed = (fabsf(*(ctrl->target_angle) - ctrl->last_target_angle) > 0.1f);
        if (target_changed) {
            ctrl->last_target_angle = *(ctrl->target_angle);
            ctrl->controlling = false; // 重置控制状态
        }

        // 位置读取逻辑 - 强制确保持续读取位置
        bool should_read_position = !ctrl->position_reading &&
                                   ((now - ctrl->last_read_tick) >= pdMS_TO_TICKS(POSITION_READ_PERIOD_MS));

        if (should_read_position) {
            // 修复：强制清除可能卡住的信号量状态
            if (ctrl->position_reading) {
                ctrl->position_reading = false;
            }

            if (StepperCAN_ReadPosition(ctrl->motor->frame_id)) {
                ctrl->last_read_tick = now;
                ctrl->position_reading = true;

                // // 添加调试信息
                // if (active_motor == 0) {
                //     printf("YAW Motor Position Valid: %d, Angle: %.2f\r\n",
                //            yaw_motor_Info.position_valid, yaw_motor_Info.current_angle);
                // } else {
                //     printf("PITCH Motor Position Valid: %d, Angle: %.2f\r\n",
                //            pitch_motor_Info.position_valid, pitch_motor_Info.current_angle);
                // }
            } else {
                // 读取失败，重置状态以便下次尝试
                ctrl->position_reading = false;
                ctrl->last_read_tick = now - pdMS_TO_TICKS(POSITION_READ_PERIOD_MS / 2); // 减少等待时间
            }
        }

        // 检查运动是否完成（针对新版电机）
        if (!ctrl->motor->ready && ctrl->motor->position_valid) {
            if (is_motor_in_position(ctrl)) {
                ctrl->motor->ready = true;  // 标记运动完成
                ctrl->controlling = false;
            }
            // 运动超时检查（防止卡死）
            else if ((now - ctrl->motion_start_tick) > pdMS_TO_TICKS(5000)) { // 5秒超时
                ctrl->motor->ready = true;  // 强制标记为完成
                ctrl->controlling = false;
            }
        }

        // 控制逻辑检查
        if (ctrl->motor->position_valid && ctrl->motor->ready &&
            !ctrl->controlling && (now - ctrl->last_control_tick) >= pdMS_TO_TICKS(CONTROL_CHECK_PERIOD_MS)) {

            float current_angle = ctrl->motor->current_angle;
            float angle_diff = angle_difference(*(ctrl->target_angle), current_angle);

            // 只有角度差值超过阈值时才执行控制
            if (fabsf(angle_diff) > ANGLE_THRESHOLD) {
                // 根据角度差值调整控制速度
                uint16_t control_speed = (uint16_t)(MAX_CONTROL_SPEED *
                    fminf(fabsf(angle_diff) / 45.0f, 1.0f));
                if (control_speed < 0x80) control_speed = 0x80; // 最小速度

                StepperCAN_SetAbsolutePosition(ctrl->motor, *(ctrl->target_angle),
                                             control_speed, ACCELERATION);
                ctrl->last_control_tick = now;
                ctrl->motion_start_tick = now;
                ctrl->controlling = true;
            }
        }

        // 重置位置读取标志位 - 修正：立即重置，避免卡住
        if (ctrl->position_reading) {
            // 检查是否已经超时，超时则强制重置
            if ((now - ctrl->last_read_tick) > pdMS_TO_TICKS(200)) { // 200ms超时
                ctrl->position_reading = false;
            }
        }

        // 交替处理两个电机以分散负载
        active_motor = (active_motor + 1) % 2;

        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    }
}
