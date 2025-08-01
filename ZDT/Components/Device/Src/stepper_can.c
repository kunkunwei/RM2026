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
extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;
extern osSemaphoreId can_cmd_mutexHandle; // 新增
/* Motor info definitions */
StepperMotor_Info_t yaw_motor_Info = {
    .frame_id = CAN1_YAW_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16,
    .ready = true
};

StepperMotor_Info_t pitch_motor_Info = {
    .frame_id = CAN1_PITCH_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16,
    .ready = true
};
/* Private Variables */
static uint8_t last_cmd[16];
static uint8_t last_cmd_len = 0;
static StepperMotorID_e last_motor_id = 0;
static uint8_t is_multi_packet = 0;


// 全局互斥标志
extern volatile bool motor_cmd_busy;

/**
  * @brief  Sends a CAN command with retry mechanism
  * @param  motor_id: Target motor ID
  * @param  cmd: Command data buffer
  * @param  len: Command length (1-16 bytes)
  * @retval true if response received, false otherwise
  */
// bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len)
// {
//     if (cmd == NULL || len == 0 || len > 16) {
//         return false;
//     }
//
//     last_motor_id = motor_id;
//     last_cmd_len = len;
//     memcpy(last_cmd, cmd, len);
//     is_multi_packet = (len > 8) ? 1 : 0;
//
//     uint8_t retries = CAN_MAX_RETRIES;
//     bool response_received = false;
//
//     while (retries-- && !response_received) {
//         while (osSemaphoreWait(can_cmd_semHandle, 0) == osOK) {}
//
//         if (len <= 8) {
//             CAN_TxFrameTypeDef tx_frame = {
//                 .hcan = &hcan1,
//                 .header = {
//                     .ExtId = motor_id,  // 直接用motor_id
//                     .IDE = CAN_ID_EXT,
//                     .RTR = CAN_RTR_DATA,
//                     .DLC = len
//                 }
//             };
//             memcpy(tx_frame.Data, cmd, len);
//             USER_CAN_TxMessage(&tx_frame);
//         } else {
//             CAN_TxFrameTypeDef tx_frame1 = {
//                 .hcan = &hcan1,
//                 .header = {
//                     .ExtId = motor_id,  // 首帧
//                     .IDE = CAN_ID_EXT,
//                     .RTR = CAN_RTR_DATA,
//                     .DLC = 8
//                 }
//             };
//             memcpy(tx_frame1.Data, cmd, 8);
//             USER_CAN_TxMessage(&tx_frame1);
//             osDelay(CAN_PACKET_DELAY_MS);
//
//             uint8_t remaining = len - 8;
//             CAN_TxFrameTypeDef tx_frame2 = {
//                 .hcan = &hcan1,
//                 .header = {
//                     .ExtId = motor_id + 1,  // 第二帧
//                     .IDE = CAN_ID_EXT,
//                     .RTR = CAN_RTR_DATA,
//                     .DLC = remaining
//                 }
//             };
//             memcpy(tx_frame2.Data, cmd + 8, remaining);
//             USER_CAN_TxMessage(&tx_frame2);
//         }
//
//         if (osSemaphoreWait(can_cmd_semHandle, CAN_SEM_WAIT_TIMEOUT) == osOK) {
//             response_received = true;
//         } else {
//             osDelay(CAN_RETRY_DELAY_MS);
//         }
//     }
//
//     return response_received;
// }
static osSemaphoreId get_motor_sem_handle(StepperMotorID_e motor_id) {
    // motor_id 可能是 uint16_t 或 enum，确保与 CAN1_YAW_MOTOR_ID/CAN1_PITCH_MOTOR_ID 类型一致
    if (motor_id == CAN1_YAW_MOTOR_ID) return can_cmd_sem_yawHandle;
    if (motor_id == CAN1_PITCH_MOTOR_ID) return can_cmd_sem_pitchHandle;
    return NULL;
}

bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len)
{
    osSemaphoreId sem = get_motor_sem_handle(motor_id);
    if (!sem) return false;

    if (cmd == NULL || len == 0 || len > 8) {
        return false;
    }

    // 发送前先获取信号量，只有获取到才允许发送
    if (osSemaphoreWait(sem, 0) != osOK) {
        return false;
    }

    bool response_received = false;
    int retry_count = 0;
    int max_retry = 2; // 默认最多重发1次

    // 判断是否为读取位置指令（0x36），如果是则不重发
    if (cmd[0] == 0x36 && len == 2) {
        max_retry = 1; // 只发一次，不重发
    }

    while (!response_received && retry_count++ < max_retry) {
        CAN_TxFrameTypeDef tx_frame = {
            .hcan = &hcan1,
            .header = {
                .StdId = motor_id,
                .IDE = CAN_ID_STD,
                .RTR = CAN_RTR_DATA,
                .DLC = len
            }
        };
        memcpy(tx_frame.Data, cmd, len);
        USER_CAN_TxMessage(&tx_frame);

        if (osSemaphoreWait(sem, CAN_SEM_WAIT_TIMEOUT) == osOK) {
            response_received = true;
        } else if (max_retry > 1) {
            osDelay(CAN_RETRY_DELAY_MS);
        }
    }

    // 如果没有收到应答，主动释放信号量，防止死锁
    if (!response_received) {
        osSemaphoreRelease(sem);
    }

    return response_received;
}
/*---------------------------------------------电机基本控制函数---------------------------------------------------------*/
/* Motor Control Functions */
void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable)
{
    uint8_t cmd[3] = {
        0xF3,
        enable ? 0x01 : 0x00,
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_SetSpeed(StepperMotor_Info_t *motor, int16_t speed, uint8_t acc)
{
    uint16_t abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > 0x4FF) abs_speed = 0x4FF;
    uint8_t direction = (speed >= 0) ? 1 : 0;
    uint8_t cmd[5] = {
        0xF6,
        (uint8_t)(((direction & 0x0F) << 4) | ((abs_speed >> 8) & 0x0F)),
        (uint8_t)(abs_speed & 0xFF),
        acc,
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_SetPositionAngle(StepperMotor_Info_t *motor, float angle, uint16_t speed, uint8_t acc, uint8_t abs_mode)
{
    // 注意：本函数 angle 为“相对运动角度”，不是绝对角度
    // 若需绝对位置控制，请在应用层计算目标角度与当前角度的差值后传入
    uint16_t abs_speed = (speed > 0x4FF) ? 0x4FF : speed;
    uint8_t direction = (angle >= 0) ? 1 : 0;
    float abs_angle = (angle >= 0) ? angle : -angle;
    // 16细分，1.8°步距角，1圈=3200脉冲
    uint32_t pulse = (uint32_t)((abs_angle * motor->subdiv * (360.0f / motor->step_angle) / 360.0f) + 0.5f);

    uint8_t cmd[8] = {
        0xFD,
        (uint8_t)(((direction & 0x0F) << 4) | ((abs_speed >> 8) & 0x0F)),
        (uint8_t)(abs_speed & 0xFF),
        acc,
        (uint8_t)((pulse >> 16) & 0xFF),
        (uint8_t)((pulse >> 8) & 0xFF),
        (uint8_t)(pulse & 0xFF),
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

void StepperCAN_EmergencyStop(StepperMotorID_e motor_id)
{
    // 立即停止命令，速度为0，加速度255，脉冲为0
    uint8_t cmd[8] = {0xFD, 0x10, 0x00, 0xFF, 0x00, 0x00, 0x00, CAN_CHECK_CODE};
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

void StepperCAN_SetZero(StepperMotorID_e motor_id)
{
    uint8_t cmd[3] = {0x0A, 0x6D, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

void StepperCAN_UnlockStallProtection(StepperMotorID_e motor_id)
{
    uint8_t cmd[4] = {0x0E, 0x52, CAN_CHECK_CODE, 0x00}; // 0x00填充，实际只用前三字节
    StepperCAN_SendCommand(motor_id, cmd, 3);
}

bool StepperCAN_ReadEncoder(StepperMotorID_e motor_id, uint16_t *value)
{
    uint8_t cmd[2] = {0x30, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadEncoder_Response 处理
    return true;
}

bool StepperCAN_ReadPulseCount(StepperMotorID_e motor_id, int32_t *count)
{
    uint8_t cmd[2] = {0x33, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadPulseCount_Response 处理
    return true;
}

bool StepperCAN_ReadPosition(StepperMotorID_e motor_id, int32_t *pos)
{
    uint8_t cmd[2] = {0x36, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadPosition_Response 处理
    // 若pos非NULL，可在此赋值（但建议直接用全局变量）
    // if (pos) *pos = ...;
    return true;
}

bool StepperCAN_ReadError(StepperMotorID_e motor_id, int16_t *error)
{
    uint8_t cmd[2] = {0x39, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadError_Response 处理
    return true;
}

bool StepperCAN_ReadEnableStatus(StepperMotorID_e motor_id, uint8_t *status)
{
    uint8_t cmd[2] = {0x3A, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadEnableStatus_Response 处理
    return true;
}

bool StepperCAN_ReadStallFlag(StepperMotorID_e motor_id, uint8_t *flag)
{
    uint8_t cmd[2] = {0x3E, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadStallFlag_Response 处理
    return true;
}

bool StepperCAN_ReadAutoHomeStatus(StepperMotorID_e motor_id, uint8_t *status)
{
    uint8_t cmd[2] = {0x3F, CAN_CHECK_CODE};
    if (!StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd))) return false;
    // 反馈由 StepperCAN_ReadAutoHomeStatus_Response 处理
    return true;
}
