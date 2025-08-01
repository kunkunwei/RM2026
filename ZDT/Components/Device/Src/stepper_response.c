//
// Created by kun on 25-7-28.
//

#include "stepper_response.h"
#include "cmsis_os.h"
#include "string.h"
#include "stepper_can.h"

extern osSemaphoreId can_cmd_semHandle;
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

// 辅助函数：根据电机ID返回对应信号量句柄（提前声明/实现）
static osSemaphoreId get_motor_sem_handle(uint16_t frame_id) {
    if (frame_id == CAN1_YAW_MOTOR_ID) return can_cmd_sem_yawHandle;
    if (frame_id == CAN1_PITCH_MOTOR_ID) return can_cmd_sem_pitchHandle;
    return NULL;
}

/**
  * @brief  Validates response format and check code
  * @param  data: Response data
  * @param  len: Response length
  * @param  expected_cmd: Expected command code
  * @retval Response status
  */
static ResponseStatus_e ValidateResponse(uint8_t *data, uint8_t len, uint8_t expected_cmd)
{
    if (len < 2) return RESPONSE_ERROR;

    /* Check for error response */
    if (data[0] == 0x00 && data[1] == 0xEE) {
        return RESPONSE_ERROR;
    }

    /* Check command code */
    if (data[0] != expected_cmd) {
        return RESPONSE_ERROR;
    }

    /* Check status code */
    switch(data[1]) {
        case RESPONSE_SUCCESS: return RESPONSE_SUCCESS;
        case RESPONSE_CONDITION_NOT_MET: return RESPONSE_CONDITION_NOT_MET;
        default: return RESPONSE_ERROR;
    }
}

void StepperCAN_Enable_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    if (ValidateResponse(data, len, 0xF3) == RESPONSE_SUCCESS) {
        osSemaphoreId sem = get_motor_sem_handle(frame_id);
        if (sem) osSemaphoreRelease(sem);
    }
}

void StepperCAN_SetSpeed_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    if (ValidateResponse(data, len, 0xF6) == RESPONSE_SUCCESS) {
        osSemaphoreId sem = get_motor_sem_handle(frame_id);
        if (sem) osSemaphoreRelease(sem);
    }
}

// 需声明外部变量
extern volatile bool motor_cmd_busy;

// 新增全局变量用于标记等待角度完成
extern uint8_t yaw_wait_angle_done;
extern uint8_t pitch_wait_angle_done;

void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    osSemaphoreId sem = get_motor_sem_handle(frame_id);

    // 运动命令完成反馈：XX 9F 6B
    if (len == 3 && data[1] == 0x9F && data[2] == CAN_CHECK_CODE) {
        if (frame_id == CAN1_YAW_MOTOR_ID) {
            yaw_motor_Info.ready = true;
            yaw_wait_angle_done = 0; // 运动完成，允许继续控制
        }
        else if (frame_id == CAN1_PITCH_MOTOR_ID) {
            pitch_motor_Info.ready = true;
            pitch_wait_angle_done = 0; // 运动完成，允许继续控制
        }
        if (sem) osSemaphoreRelease(sem);
    }
    // 命令接收应答：XX 02 6B
    else if (len == 3 && data[1] == 0x02 && data[2] == CAN_CHECK_CODE) {
        // 对于0角度命令，只有这一个应答，直接清除等待标志
        if (frame_id == CAN1_YAW_MOTOR_ID) {
            // 检查是否为0角度命令（可以通过其他方式判断）
            yaw_wait_angle_done = 0;
        }
        else if (frame_id == CAN1_PITCH_MOTOR_ID) {
            pitch_wait_angle_done = 0;
        }
        if (sem) osSemaphoreRelease(sem);
    }
}

void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    if (ValidateResponse(data, len, 0xFE) == RESPONSE_SUCCESS) {
        osSemaphoreId sem = get_motor_sem_handle(frame_id);
        if (sem) osSemaphoreRelease(sem);
    }
}

void StepperCAN_SetSingleZero_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    if (ValidateResponse(data, len, 0x93) == RESPONSE_SUCCESS) {
        osSemaphoreId sem = get_motor_sem_handle(frame_id);
        if (sem) osSemaphoreRelease(sem);
    }
}

void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len, uint16_t frame_id)
{
    if (ValidateResponse(data, len, 0x9A) == RESPONSE_SUCCESS) {
        osSemaphoreId sem = get_motor_sem_handle(frame_id);
        if (sem) osSemaphoreRelease(sem);
    }
}

void StepperCAN_ReadSystemStatus_Response(uint16_t frame_id, uint8_t *data, uint16_t len)
{
    /* Check if this is a system status response */
    if (len < 2 || data[0] != 0x43) {
        return;
    }

    /* Check if it's the final packet */
    if (len == 2 && data[1] == CAN_CHECK_CODE) {
        osSemaphoreRelease(can_cmd_semHandle);
        return;
    }

    /* Process system status data */
    Stepper_motor_measure_t *motor = (frame_id == CAN1_YAW_MOTOR_ID) ?
        get_yaw_motor_info() : get_pitch_motor_info();

    if (motor != NULL) {
        get_stepper_system_status_measure(motor, data);
    }
}

// 全局变量用于保存反馈数据
uint16_t g_encoder_value_yaw = 0, g_encoder_value_pitch = 0;
int32_t g_pulse_count_yaw = 0, g_pulse_count_pitch = 0;
int32_t g_position_yaw = 0, g_position_pitch = 0;
int16_t g_error_yaw = 0, g_error_pitch = 0;
uint8_t g_enable_yaw = 0, g_enable_pitch = 0;
uint8_t g_stall_yaw = 0, g_stall_pitch = 0;
uint8_t g_home_status_yaw = 0, g_home_status_pitch = 0;

// 编码器反馈
void StepperCAN_ReadEncoder_Response(uint16_t frame_id, uint16_t encoder)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_encoder_value_yaw = encoder;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_encoder_value_pitch = encoder;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 输入脉冲数反馈
void StepperCAN_ReadPulseCount_Response(uint16_t frame_id, int32_t pulse)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_pulse_count_yaw = pulse;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_pulse_count_pitch = pulse;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 实时位置反馈
void StepperCAN_ReadPosition_Response(uint16_t frame_id, int32_t position)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_position_yaw = position;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_position_pitch = position;

    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 位置误差反馈
void StepperCAN_ReadError_Response(uint16_t frame_id, int16_t error)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_error_yaw = error;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_error_pitch = error;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 使能状态反馈
void StepperCAN_ReadEnableStatus_Response(uint16_t frame_id, uint8_t enable)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_enable_yaw = enable;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_enable_pitch = enable;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 堵转标志反馈
void StepperCAN_ReadStallFlag_Response(uint16_t frame_id, uint8_t stall)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_stall_yaw = stall;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_stall_pitch = stall;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 自动回零状态反馈
void StepperCAN_ReadAutoHomeStatus_Response(uint16_t frame_id, uint8_t home_status)
{
    if (frame_id == CAN1_YAW_MOTOR_ID)
        g_home_status_yaw = home_status;
    else if (frame_id == CAN1_PITCH_MOTOR_ID)
        g_home_status_pitch = home_status;
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 设置零点反馈
void StepperCAN_SetZero_Response(uint16_t frame_id)
{
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}

// 解除堵转保护反馈
void StepperCAN_UnlockStallProtection_Response(uint16_t frame_id)
{
    osSemaphoreId sem = get_motor_sem_handle(frame_id);
    if (sem) osSemaphoreRelease(sem);
}
