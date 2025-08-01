//
// Created by kun on 25-7-28.
//

#include "stepper_response.h"
#include "cmsis_os.h"
#include "string.h"
#include "stepper_can.h"

extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

// 位置数据存储
int32_t g_position_yaw = 0, g_position_pitch = 0;

// 辅助函数：获取信号量句柄
static osSemaphoreId get_motor_sem_handle(uint16_t frame_id) {
    if (frame_id == CAN1_YAW_MOTOR_ID) return can_cmd_sem_yawHandle;
    if (frame_id == CAN1_PITCH_MOTOR_ID) return can_cmd_sem_pitchHandle;
    return NULL;
}

// 辅助函数：更新电机角度信息
static void update_motor_angle(uint16_t frame_id, int32_t position) {
    float angle = (float)position * 360.0f / 65536.0f;

    if (frame_id == CAN1_YAW_MOTOR_ID) {
        yaw_motor_Info.current_angle = angle;
        yaw_motor_Info.position_valid = true;
        g_position_yaw = position;
    } else if (frame_id == CAN1_PITCH_MOTOR_ID) {
        pitch_motor_Info.current_angle = angle;
        pitch_motor_Info.position_valid = true;
        g_position_pitch = position;
    }
}

/**
  * @brief  验证响应格式和校验码
  */
static ResponseStatus_e ValidateResponse(uint8_t *data, uint8_t len, uint8_t expected_cmd)
{
    if (len < 2) return RESPONSE_ERROR;

    // 检查错误响应
    if (data[0] == 0x00 && data[1] == 0xEE) {
        return RESPONSE_ERROR;
    }

    // 检查命令码
    if (data[0] != expected_cmd) {
        return RESPONSE_ERROR;
    }

    // 检查状态码
    switch(data[1]) {
        case RESPONSE_SUCCESS: return RESPONSE_SUCCESS;
        case RESPONSE_CONDITION_NOT_MET: return RESPONSE_CONDITION_NOT_MET;
        default: return RESPONSE_ERROR;
    }
}

/* 使能响应处理 */
void StepperCAN_Enable_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) {
        osSemaphoreRelease(sem);
    }
}

/* 位置控制响应处理 */
void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    osSemaphoreId sem = get_motor_sem_handle(frame_id);

    // 命令接收应答：XX FD 02 6B - 立即标记电机为运动状态
    if (len >= 3 && data[0] == 0xFD && data[1] == 0x02) {
        // 新版电机只发送这一帧反馈，运动完成后需要通过位置读取来判断是否到位
        if (frame_id == CAN1_YAW_MOTOR_ID) {
            yaw_motor_Info.ready = false; // 标记为运动中，需要通过位置检查来确认完成
        } else if (frame_id == CAN1_PITCH_MOTOR_ID) {
            pitch_motor_Info.ready = false; // 标记为运动中，需要通过位置检查来确认完成
        }
    }

    if (sem) {
        osSemaphoreRelease(sem);
    }
}

/* 紧急停止响应处理 */
void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    // 停止后标记电机可用
    if (frame_id == CAN1_YAW_MOTOR_ID) {
        yaw_motor_Info.ready = true;
    } else if (frame_id == CAN1_PITCH_MOTOR_ID) {
        pitch_motor_Info.ready = true;
    }

    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) {
        osSemaphoreRelease(sem);
    }
}

/* 回零响应处理 */
void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    // 回零完成后重置角度
    if (frame_id == CAN1_YAW_MOTOR_ID) {
        yaw_motor_Info.current_angle = 0.0f;
        yaw_motor_Info.ready = true;
        g_position_yaw = 0;
    } else if (frame_id == CAN1_PITCH_MOTOR_ID) {
        pitch_motor_Info.current_angle = 0.0f;
        pitch_motor_Info.ready = true;
        g_position_pitch = 0;
    }

    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) {
        osSemaphoreRelease(sem);
    }
}

/* 实时位置反馈处理 */
void StepperCAN_ReadPosition_Response(uint16_t frame_id, uint8_t sign, int32_t position_raw)
{
    // 处理符号位
    int32_t position = sign ? -position_raw : position_raw;

    // 更新电机角度信息
    update_motor_angle(frame_id, position);

    // 释放信号量，允许继续发送命令
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) {
        osSemaphoreRelease(sem);
    }
}
