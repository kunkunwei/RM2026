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

// 引用 stepper_response.c 的全局变量
extern int32_t g_position_yaw, g_position_pitch;
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

void Gimbal_Task(void const * argument)
{
    osDelay(1000);
    StepperCAN_Enable(&yaw_motor_Info, true);
    StepperCAN_Enable(&pitch_motor_Info, true);

    TickType_t systick = 0;
    float target_yaw_angle = 0;
    float target_pitch_angle = 0;

    uint16_t yaw_ready_cnt = 0, pitch_ready_cnt = 0;
    uint8_t yaw_dead_cnt = 0, pitch_dead_cnt = 0;
    const uint16_t READY_TIMEOUT_CNT = 200 / CONTROL_PERIOD_MS; // 200ms超时
    const uint8_t DEAD_RETRY_MAX = 2; // 超时2次后判定死机
    const uint16_t DEAD_RETRY_DELAY = 300; // 死机恢复后延时(ms)

    // 新增：信号量超时计数器
    uint16_t yaw_sem_timeout_cnt = 0, pitch_sem_timeout_cnt = 0;
    const uint16_t SEM_TIMEOUT_CNT = 500 / CONTROL_PERIOD_MS; // 500ms超时

    #define CMD_INTERVAL_MS 30
    TickType_t last_yaw_cmd_tick = 0;
    TickType_t last_pitch_cmd_tick = 0;

    // 新增：轮询优先标志，0=YAW优先，1=PITCH优先
    uint8_t motor_priority = 0;

    for(;;)
    {
        systick = osKernelSysTick();

        float yaw_actual_angle = (float)(g_position_yaw) * 360.0f / 65536.0f;
        float pitch_actual_angle = (float)(g_position_pitch) * 360.0f / 65536.0f;

        int16_t rc_yaw_raw = remote_ctrl.rc.ch[0];
        int16_t rc_pitch_raw = remote_ctrl.rc.ch[1];

        // 直接映射遥控器通道值到角度范围
        target_yaw_angle = ((float)rc_yaw_raw) * (YAW_ANGLE_MAX - YAW_ANGLE_MIN) / (RC_CH_MAX - RC_CH_MIN);
        target_pitch_angle = ((float)rc_pitch_raw) * (PITCH_ANGLE_MAX - PITCH_ANGLE_MIN) / (RC_CH_MAX - RC_CH_MIN);

        TickType_t now = osKernelSysTick();

        // 优先轮流下发控制指令，避免一个电机连续多次下发
        if (motor_priority == 0) {
            if (yaw_motor_Info.ready && (now - last_yaw_cmd_tick >= CMD_INTERVAL_MS)) {
                float yaw_delta = target_yaw_angle - yaw_actual_angle;
                StepperCAN_SetPositionAngle(&yaw_motor_Info, yaw_delta, 0x40, 0x20, 0);
                yaw_motor_Info.ready = false;
                yaw_ready_cnt = 0;
                last_yaw_cmd_tick = now;
                yaw_sem_timeout_cnt = 0;
                motor_priority = 1; // 下次优先pitch
            } else if (pitch_motor_Info.ready && (now - last_pitch_cmd_tick >= CMD_INTERVAL_MS)) {
                float pitch_delta = target_pitch_angle - pitch_actual_angle;
                StepperCAN_SetPositionAngle(&pitch_motor_Info, pitch_delta, 0x40, 0x20, 0);
                pitch_motor_Info.ready = false;
                pitch_ready_cnt = 0;
                last_pitch_cmd_tick = now;
                pitch_sem_timeout_cnt = 0;
                motor_priority = 0; // 下次优先yaw
            }
        } else {
            if (pitch_motor_Info.ready && (now - last_pitch_cmd_tick >= CMD_INTERVAL_MS)) {
                float pitch_delta = target_pitch_angle - pitch_actual_angle;
                StepperCAN_SetPositionAngle(&pitch_motor_Info, pitch_delta, 0x40, 0x20, 0);
                pitch_motor_Info.ready = false;
                pitch_ready_cnt = 0;
                last_pitch_cmd_tick = now;
                pitch_sem_timeout_cnt = 0;
                motor_priority = 0; // 下次优先yaw
            } else if (yaw_motor_Info.ready && (now - last_yaw_cmd_tick >= CMD_INTERVAL_MS)) {
                float yaw_delta = target_yaw_angle - yaw_actual_angle;
                StepperCAN_SetPositionAngle(&yaw_motor_Info, yaw_delta, 0x40, 0x20, 0);
                yaw_motor_Info.ready = false;
                yaw_ready_cnt = 0;
                last_yaw_cmd_tick = now;
                yaw_sem_timeout_cnt = 0;
                motor_priority = 1; // 下次优先pitch
            }
        }

        // YAW轴死机自恢复（去掉失能/使能，仅恢复信号量和ready）
        if (!yaw_motor_Info.ready)
        {
            yaw_ready_cnt++;
            yaw_sem_timeout_cnt++;
            if (yaw_ready_cnt > READY_TIMEOUT_CNT) {
                yaw_ready_cnt = 0;
                yaw_dead_cnt++;
                if (yaw_dead_cnt >= DEAD_RETRY_MAX) {
                    // 暂不做失能/使能，仅恢复ready
                    yaw_motor_Info.ready = true;
                    yaw_dead_cnt = 0;
                }
            }
            // 信号量超时自恢复（不再失能/使能，只释放信号量和重置ready）
            if (yaw_sem_timeout_cnt > SEM_TIMEOUT_CNT) {
                osSemaphoreRelease(can_cmd_sem_yawHandle);
                yaw_motor_Info.ready = true;
                yaw_sem_timeout_cnt = 0;
            }
        } else {
            yaw_dead_cnt = 0;
            yaw_sem_timeout_cnt = 0;
        }

        // PITCH轴死机自恢复（去掉失能/使能，仅恢复信号量和ready）
        if (!pitch_motor_Info.ready)
        {
            pitch_ready_cnt++;
            pitch_sem_timeout_cnt++;
            if (pitch_ready_cnt > READY_TIMEOUT_CNT) {
                pitch_ready_cnt = 0;
                pitch_dead_cnt++;
                if (pitch_dead_cnt >= DEAD_RETRY_MAX) {
                    pitch_motor_Info.ready = true;
                    pitch_dead_cnt = 0;
                }
            }
            // 信号量超时自恢复（不再失能/使能，只释放信号量和重置ready）
            if (pitch_sem_timeout_cnt > SEM_TIMEOUT_CNT) {
                osSemaphoreRelease(can_cmd_sem_pitchHandle);
                pitch_motor_Info.ready = true;
                pitch_sem_timeout_cnt = 0;
            }
        } else {
            pitch_dead_cnt = 0;
            pitch_sem_timeout_cnt = 0;
        }

        osDelayUntil(&systick, CONTROL_PERIOD_MS);
    }
    // 永远不会到这里，防止意外退出
    while(1) { osDelay(1000); }
}

// 串口接收相关代码已移除，CAN反馈处理请在 stepper_response.c/h 实现
