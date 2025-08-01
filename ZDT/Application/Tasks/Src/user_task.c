//
// Created by kun on 25-7-13.
//

#include "../Inc/user_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "stepper_can.h"
#include "usart.h"

extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;
extern osThreadId StartUserTaskHandle;

uint8_t yaw_wait_angle_done = 0;
uint8_t pitch_wait_angle_done = 0;

extern float target_yaw_angle, target_pitch_angle;
extern int32_t g_position_yaw, g_position_pitch;

#define READ_PERIOD_MS 20   // 读取周期20ms
#define CTRL_INTERVAL_MS 50 // 控制指令最小间隔50ms
#define ANGLE_THRESHOLD 0.5f // 角度差值阈值

typedef struct {
    enum {
        IDLE,           // 空闲状态
        READING_POS,    // 正在读取位置
        CONTROLLING     // 正在执行控制
    } state;
    StepperMotor_Info_t* info;
    float* target_angle;
    int32_t* position;
    uint8_t* wait_angle_done;
    TickType_t last_read_tick;
    TickType_t last_ctrl_tick;
} MotorFSM_t;

// 角度转换函数
static float position_to_angle(int32_t position) {
    return (float)position * 360.0f / 65536.0f;
}

void User_Task(void const * argument)
{
    MotorFSM_t motors[2] = {
        {IDLE, &yaw_motor_Info, &target_yaw_angle, &g_position_yaw, &yaw_wait_angle_done, 0, 0},
        {IDLE, &pitch_motor_Info, &target_pitch_angle, &g_position_pitch, &pitch_wait_angle_done, 0, 0}
    };

    uint8_t current_motor = 0; // 当前处理的电机，实现交替控制

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        TickType_t now = osKernelSysTick();
        MotorFSM_t* m = &motors[current_motor];

        switch (m->state) {
        case IDLE:
            // 检查是否可以开始新的读取周期
            if (*(m->wait_angle_done) == 0 && (now - m->last_read_tick) >= READ_PERIOD_MS) {
                if (StepperCAN_ReadPosition(m->info->frame_id, NULL)) {
                    m->last_read_tick = now;
                    m->state = READING_POS;
                }
            }
            break;

        case READING_POS:
            // 读取位置后，检查是否需要控制
            if (*(m->wait_angle_done) == 0 && (now - m->last_ctrl_tick) >= CTRL_INTERVAL_MS) {
                float actual_angle = position_to_angle(*(m->position));
                float delta = *(m->target_angle) - actual_angle;

                // 只有角度差值超过阈值时才发送控制指令
                if (fabsf(delta) > ANGLE_THRESHOLD) {
                    StepperCAN_SetPositionAngle(m->info, delta, 0x40, 0x20, 0);
                    m->last_ctrl_tick = now;
                    *(m->wait_angle_done) = 1; // 标记正在运动
                    m->state = CONTROLLING;
                } else {
                    m->state = IDLE; // 角度已到位，回到空闲状态
                }
            } else {
                m->state = IDLE; // 回到空闲状态
            }
            break;

        case CONTROLLING:
            // 等待运动完成
            if (*(m->wait_angle_done) == 0) {
                m->state = IDLE;
            }
            break;
        }

        // 交替处理两个电机
        current_motor = (current_motor + 1) % 2;

        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    }
    while(1) { osDelay(1000); }
}
