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
    void StepperCAN_Enable_Response(uint8_t *data, uint8_t len);
    void StepperCAN_SetSpeed_Response(uint8_t *data, uint8_t len);
    void StepperCAN_SetPositionAngle_Response(uint8_t *data, uint8_t len);
    void StepperCAN_EmergencyStop_Response(uint8_t *data, uint8_t len);
    void StepperCAN_SetSingleZero_Response(uint8_t *data, uint8_t len);
    void StepperCAN_TriggerHome_Response(uint8_t *data, uint8_t len);
    void StepperCAN_ReadSystemStatus_Response(uint16_t frame_id, uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif //STEPPER_RESPONSE_H
