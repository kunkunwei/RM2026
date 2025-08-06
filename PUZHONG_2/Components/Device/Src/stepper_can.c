//
// Created by kun on 25-7-27.
//

/* USER CODE END Header */

#include "stepper_can.h"
// #include "freertos_config.h"
#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "string.h"
extern osSemaphoreId can_cmd_semHandle;
/* Motor info definitions */
StepperMotor_Info_t yaw_motor_Info = {
    .frame_id = CAN1_YAW_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16
};

StepperMotor_Info_t pitch_motor_Info = {
    .frame_id = CAN1_PITCH_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16
};
/* Private Variables */
static uint8_t last_cmd[16];
static uint8_t last_cmd_len = 0;
static StepperMotorID_e last_motor_id = 0;
static uint8_t is_multi_packet = 0;


/**
  * @brief  Sends a CAN command with retry mechanism
  * @param  motor_id: Target motor ID
  * @param  cmd: Command data buffer
  * @param  len: Command length (1-16 bytes)
  * @retval true if response received, false otherwise
  */
bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL || len == 0 || len > 16) {
        return false;
    }

    /* Store command for retry */
    last_motor_id = motor_id;
    last_cmd_len = len;
    memcpy(last_cmd, cmd, len);
    is_multi_packet = (len > 8) ? 1 : 0;

    uint8_t retries = CAN_MAX_RETRIES;
    bool response_received = false;

    while (retries-- && !response_received) {
        /* Clear semaphore */
        while (osSemaphoreWait(can_cmd_semHandle, 0) == osOK) {}

        /* Send command */
        if (len <= 8) {
            /* Single packet */
            CAN_TxFrameTypeDef tx_frame = {
                .hcan = &hcan1,
                .header = {
                    .ExtId = (motor_id << 8),  // Low byte is 0 for single packet
                    .IDE = CAN_ID_EXT,
                    .RTR = CAN_RTR_DATA,
                    .DLC = len
                }
            };
            memcpy(tx_frame.Data, cmd, len);
            USER_CAN_TxMessage(&tx_frame);
        } else {
            /* Multi-packet */
            // First packet (8 bytes)
            CAN_TxFrameTypeDef tx_frame1 = {
                .hcan = &hcan1,
                .header = {
                    .ExtId = (motor_id << 8),  // Low byte is 0 for first packet
                    .IDE = CAN_ID_EXT,
                    .RTR = CAN_RTR_DATA,
                    .DLC = 8
                }
            };
            memcpy(tx_frame1.Data, cmd, 8);
            USER_CAN_TxMessage(&tx_frame1);
            osDelay(CAN_PACKET_DELAY_MS);

            // Second packet (remaining bytes)
            uint8_t remaining = len - 8;
            CAN_TxFrameTypeDef tx_frame2 = {
                .hcan = &hcan1,
                .header = {
                    .ExtId = (motor_id << 8) | 0x01,  // Low byte is 1 for second packet
                    .IDE = CAN_ID_EXT,
                    .RTR = CAN_RTR_DATA,
                    .DLC = remaining
                }
            };
            memcpy(tx_frame2.Data, cmd + 8, remaining);
            USER_CAN_TxMessage(&tx_frame2);
        }

        /* Wait for response */
        if (osSemaphoreWait(can_cmd_semHandle, CAN_SEM_WAIT_TIMEOUT) == osOK) {
            response_received = true;
        } else {
            /* Delay before retry */
            osDelay(CAN_RETRY_DELAY_MS);
        }
    }

    return response_received;
}


/*---------------------------------------------电机基本控制函数---------------------------------------------------------*/
/* Motor Control Functions */
void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable)
{
    uint8_t cmd[5] = {
        0xF3, 0xAB,
        enable ? 0x01 : 0x00,
        0x00,  // Multi-motor sync flag
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_SetSpeed(StepperMotor_Info_t *motor, int16_t speed, uint8_t accel)
{
    uint8_t direction = (speed >= 0) ? 0x01 : 0x00;
    uint16_t abs_speed = (speed >= 0) ? speed : -speed;

    uint8_t cmd[7] = {
        0xF6,
        direction,
        (abs_speed >> 8) & 0xFF,
        abs_speed & 0xFF,
        accel,
        0x00,  // Multi-motor sync flag
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_SetPositionAngle(StepperMotor_Info_t *motor, float angle, uint16_t speed, uint8_t accel, uint8_t abs_mode)
{
    uint8_t direction = (angle >= 0) ? 0x01 : 0x00;
    float abs_angle = (angle >= 0) ? angle : -angle;
    float pulse_per_deg = motor->subdiv / motor->step_angle;
    uint32_t pulse = (uint32_t)(abs_angle * pulse_per_deg + 0.5f);

    uint8_t cmd[12] = {
        0xFD,
        direction,
        (speed >> 8) & 0xFF,
        speed & 0xFF,
        accel,
        (pulse >> 24) & 0xFF,
        (pulse >> 16) & 0xFF,
        (pulse >> 8) & 0xFF,
        pulse & 0xFF,
        abs_mode ? 0x01 : 0x00,
        0x00,  // Multi-motor sync flag
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_EmergencyStop(StepperMotorID_e motor_id)
{
    uint8_t cmd[4] = {0xFE, 0x98, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

void StepperCAN_SetSingleZero(StepperMotorID_e motor_id, uint8_t store_flag)
{
    uint8_t cmd[4] = {0x93, 0x88, store_flag ? 0x01 : 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode)
{
    uint8_t cmd[4] = {0x9A, (uint8_t)mode, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

void StepperCAN_ReadSystemStatus(StepperMotorID_e motor_id)
{
    uint8_t cmd[3] = {0x43, 0x7A, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}