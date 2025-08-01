//
// Created by kun on 25-7-27.
//

/* USER CODE END Header */

#include "stepper_can.h"
#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "string.h"
#include "math.h"

extern osSemaphoreId can_cmd_sem_yawHandle;
extern osSemaphoreId can_cmd_sem_pitchHandle;

/* Motor info definitions */
StepperMotor_Info_t yaw_motor_Info = {
    .frame_id = CAN1_YAW_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16,
    .ready = true,
    .current_angle = 0.0f,
    .position_valid = false
};

StepperMotor_Info_t pitch_motor_Info = {
    .frame_id = CAN1_PITCH_MOTOR_ID,
    .step_angle = 1.8f,
    .subdiv = 16,
    .ready = true,
    .current_angle = 0.0f,
    .position_valid = false
};

/* 获取电机对应的信号量 */
static osSemaphoreId get_motor_sem_handle(StepperMotorID_e motor_id) {
    if (motor_id == CAN1_YAW_MOTOR_ID) return can_cmd_sem_yawHandle;
    if (motor_id == CAN1_PITCH_MOTOR_ID) return can_cmd_sem_pitchHandle;
    return NULL;
}

/* 基础命令发送函数 - 单帧 */
bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len)
{
    osSemaphoreId sem = get_motor_sem_handle(motor_id);
    if (!sem || cmd == NULL || len == 0 || len > 8) {
        return false;
    }

    // 尝试获取信号量
    if (osSemaphoreWait(sem, 10) != osOK) {
        osSemaphoreRelease(sem);
        osDelay(2);
        if (osSemaphoreWait(sem, 10) != osOK) {
            return false;
        }
    }

    bool response_received = false;
    int retry_count = 0;

    while (!response_received && retry_count < CAN_MAX_RETRIES) {
        retry_count++;

        CAN_TxFrameTypeDef tx_frame = {
            .hcan = &hcan1,
            .header = {
                .ExtId = motor_id,
                .IDE = CAN_ID_EXT,
                .RTR = CAN_RTR_DATA,
                .DLC = len
            }
        };
        memcpy(tx_frame.Data, cmd, len);
        USER_CAN_TxMessage(&tx_frame);

        // 等待应答
        uint32_t timeout = (cmd[0] == 0x36) ? 20 : 30;
        if (osSemaphoreWait(sem, timeout) == osOK) {
            response_received = true;
        } else if (retry_count < CAN_MAX_RETRIES) {
            osDelay(2);
        }
    }

    if (!response_received) {
        osSemaphoreRelease(sem);
    }

    return response_received;
}

/* 位置控制命令发送函数 - 分包发送 */
bool StepperCAN_SendPositionCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t total_len)
{
    osSemaphoreId sem = get_motor_sem_handle(motor_id);
    if (!sem || cmd == NULL || total_len != 12) { // 位置控制命令固定12字节
        return false;
    }

    // 尝试获取信号量
    if (osSemaphoreWait(sem, 50) != osOK) {
        return false;
    }

    bool response_received = false;
    int retry_count = 0;

    while (!response_received && retry_count < CAN_MAX_RETRIES) {
        retry_count++;

        // 第一帧：8字节数据（包ID为基础ID）
        CAN_TxFrameTypeDef tx_frame1 = {
            .hcan = &hcan1,
            .header = {
                .ExtId = motor_id,  // 第0包数据
                .IDE = CAN_ID_EXT,
                .RTR = CAN_RTR_DATA,
                .DLC = 8
            }
        };
        memcpy(tx_frame1.Data, cmd, 8);
        USER_CAN_TxMessage(&tx_frame1);

        // 短延时
        osDelay(CAN_PACKET_DELAY_MS);

        // 第二帧：剩余4字节（包ID为基础ID+1）
        CAN_TxFrameTypeDef tx_frame2 = {
            .hcan = &hcan1,
            .header = {
                .ExtId = motor_id + 1,  // 第1包数据
                .IDE = CAN_ID_EXT,
                .RTR = CAN_RTR_DATA,
                .DLC = 5
            }
        };
        // memcpy(tx_frame2.Data, &cmd[8], 4);
        // tx_frame2.Data[0]=0xFD; // 位置控制命令码
        // tx_frame2.Data[1]=0;
        // tx_frame2.Data[2]=0;
        // tx_frame2.Data[3]=0;
        // tx_frame2.Data[4]=CAN_CHECK_CODE; // 校验字节
        tx_frame2.Data[0] = cmd[0]; // FD
        memcpy(&tx_frame2.Data[1], cmd + 8, 4); // 00 00 00 6B
        USER_CAN_TxMessage(&tx_frame2);

        // 等待应答
        if (osSemaphoreWait(sem, CAN_SEM_WAIT_TIMEOUT) == osOK) {
            response_received = true;
        } else if (retry_count < CAN_MAX_RETRIES) {
            osDelay(CAN_RETRY_DELAY_MS);
        }
    }

    if (!response_received) {
        osSemaphoreRelease(sem);
    }

    return response_received;
}

/*---------------------------------------------电机控制函数---------------------------------------------------------*/

/* 电机使能控制 */
void StepperCAN_Enable(StepperMotor_Info_t *motor, bool enable)
{
    uint8_t cmd[5] = {
        0xF3,
        0xAB,
        enable ? 0x01 : 0x00,
        0x00,
        CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, sizeof(cmd));
}

/* 绝对位置控制 - 修正版本，按照正确协议格式 */
void StepperCAN_SetAbsolutePosition(StepperMotor_Info_t *motor, float target_angle, uint16_t speed_rpm, uint8_t accel)
{
    // 参数验证
    if (speed_rpm > 0x4FF) speed_rpm = 0x4FF;  // 最大速度限制
    if (speed_rpm == 0) speed_rpm = 0x100;     // 默认速度256 RPM

    // 计算脉冲数：16细分，3200脉冲/圈
    uint32_t pulse = (uint32_t)((fabsf(target_angle) / 360.0f) * 3200.0f + 0.5f);

    // 确定方向：正角度为CCW(1)，负角度为CW(0)
    uint8_t direction = (target_angle >= 0) ? 0x01 : 0x00;

    // 按照协议格式构建12字节命令：
    // 地址 + 0xFD + 方向 + 速度+ 加速度 + 脉冲数 + 相对/绝对模式标志 + 多机同步标志 + 校验字节
    uint8_t cmd[12] = {
        0xFD,                                    // [0] 位置控制命令码
        direction,                               // [1] 方向：01=CCW, 00=CW
        (uint8_t)((speed_rpm >> 8) & 0xFF),     // [2] 速度高字节
        (uint8_t)(speed_rpm & 0xFF),            // [3] 速度低字节
        accel,                                  // [4] 加速度档位
        (uint8_t)((pulse >> 24) & 0xFF),        // [5] 脉冲数最高字节
        (uint8_t)((pulse >> 16) & 0xFF),        // [6] 脉冲数次高字节
        (uint8_t)((pulse >> 8) & 0xFF),         // [7] 脉冲数次低字节
        (uint8_t)(pulse & 0xFF),                // [8] 脉冲数最低字节
        0x01,                                   // [9] 绝对位置模式标志 (01=绝对, 00=相对)
        0x00,                                   // [10] 多机同步标志 (00=不启用, 01=启用)
        CAN_CHECK_CODE                          // [11] 校验字节 0x6B
    };

    // 发送分包命令
    if (StepperCAN_SendPositionCommand(motor->frame_id, cmd, sizeof(cmd))) {
        motor->ready = false; // 标记电机正在运动
    }
}

/* 紧急停止 */
void StepperCAN_EmergencyStop(StepperMotorID_e motor_id)
{
    uint8_t cmd[4] = {0xFE, 0x98, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

/* 触发回零 */
void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode)
{
    uint8_t cmd[4] = {0x9A, (uint8_t)mode, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

/* 读取实时位置 */
bool StepperCAN_ReadPosition(StepperMotorID_e motor_id)
{
    uint8_t cmd[2] = {0x36, CAN_CHECK_CODE};
    return StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}
