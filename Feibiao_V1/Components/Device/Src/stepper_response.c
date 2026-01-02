//
// Created by kun on 25-7-28.
//

#include "stepper_response.h"
#include "cmsis_os.h"
#include "string.h"

extern osSemaphoreId can_cmd_semHandle;

/**
 * @brief 校验返回应答的格式和状态码
 * @param data 应答数据指针
 * @param len 应答数据长度
 * @param expected_cmd 期望的命令码（首字节）
 * @return RESPONSE_SUCCESS 表示应答正确并返回成功状态，其他表示错误或条件不满足
 *
 * @note 应答格式约定：data[0] 为命令码，data[1] 为状态码（0x02 成功，0xE2 条件不满足，0xEE 错误）
 */
static ResponseStatus_e ValidateResponse(const uint8_t *data, uint8_t len, uint8_t expected_cmd)
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

/**
 * @brief 处理使能命令的应答（F3）
 * @param data 应答数据
 * @param len 长度
 * @note 成功时释放 can_cmd_semHandle 信号量，唤醒等待发送结果的线程
 */
void StepperCAN_Enable_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xF3) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理设置速度命令的应答（F6）
 */
void StepperCAN_SetSpeed_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xF6) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理按角度定位命令的应答（FD）
 */
void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xFD) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理紧急停止命令的应答（FE）
 */
void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xFE) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理设置单圈零点命令的应答（93）
 */
void StepperCAN_SetSingleZero_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0x93) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理触发回零命令的应答（9A）
 */
void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0x9A) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

/**
 * @brief 处理读取系统状态（多帧）应答
 * @param frame_id 源帧 ID（用于判断是 YAW/PITCH）
 * @param data 指向当前接收到的数据包内容（已经按帧处理传入）
 * @param len 当前包的长度
 *
 * @note 多帧拼接逻辑在接收端（bsp_can.c 的 handler）完成。
 *       当收到仅包含 [0x43, CAN_CHECK_CODE] 的结束包时会释放信号量表示完成。
 */
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