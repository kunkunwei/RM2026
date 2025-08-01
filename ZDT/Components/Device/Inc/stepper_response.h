//
// Created by kun on 25-7-28.
//

#ifndef STEPPER_RESPONSE_H
#define STEPPER_RESPONSE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stepper_motor.h"
#include "stepper_can.h"

    /* Response Status */
    typedef enum {
        RESPONSE_SUCCESS = 0x02,
        RESPONSE_CONDITION_NOT_MET = 0xE2,
        RESPONSE_ERROR = 0xEE
    } ResponseStatus_e;

    /* Response Handlers */
    void StepperCAN_Enable_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_SetSpeed_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_SetSingleZero_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len, uint16_t frame_id);
    void StepperCAN_ReadSystemStatus_Response(uint16_t frame_id, uint8_t *data, uint16_t len);
    void StepperCAN_ReadEncoder_Response(uint16_t frame_id, uint16_t encoder);
    void StepperCAN_ReadPulseCount_Response(uint16_t frame_id, int32_t pulse);
    void StepperCAN_ReadPosition_Response(uint16_t frame_id, int32_t position);
    void StepperCAN_ReadError_Response(uint16_t frame_id, int16_t error);
    void StepperCAN_ReadEnableStatus_Response(uint16_t frame_id, uint8_t enable);
    void StepperCAN_ReadStallFlag_Response(uint16_t frame_id, uint8_t stall);
    void StepperCAN_ReadAutoHomeStatus_Response(uint16_t frame_id, uint8_t home_status);
    void StepperCAN_SetZero_Response(uint16_t frame_id);
    void StepperCAN_UnlockStallProtection_Response(uint16_t frame_id);

    // 全局反馈变量声明
    extern uint16_t g_encoder_value_yaw, g_encoder_value_pitch;
    extern int32_t g_pulse_count_yaw, g_pulse_count_pitch;
    extern int32_t g_position_yaw, g_position_pitch;
    extern int16_t g_error_yaw, g_error_pitch;
    extern uint8_t g_enable_yaw, g_enable_pitch;
    extern uint8_t g_stall_yaw, g_stall_pitch;
    extern uint8_t g_home_status_yaw, g_home_status_pitch;

#ifdef __cplusplus
}
#endif

#endif //STEPPER_RESPONSE_H
