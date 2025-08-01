//
// Created by kun on 25-7-28.
//

#include "stepper_response.h"
#include "cmsis_os.h"
#include "string.h"

extern osSemaphoreId can_cmd_semHandle;

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

void StepperCAN_Enable_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xF3) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

void StepperCAN_SetSpeed_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xF6) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xFD) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0xFE) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

void StepperCAN_SetSingleZero_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0x93) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
    }
}

void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len)
{
    if (ValidateResponse(data, len, 0x9A) == RESPONSE_SUCCESS) {
        osSemaphoreRelease(can_cmd_semHandle);
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