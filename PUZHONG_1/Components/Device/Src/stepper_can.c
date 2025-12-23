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

/**
 * @file stepper_can.c
 * @brief 步进电机 CAN 协议接口实现（中文注释版）
 *
 * 该文件实现了通过 CAN 总线向步进电机发送指令的封装函数，
 * 支持单帧与多帧（最多两帧）发送逻辑，以及若干常用的电机控制接口。
 * 多帧发送规则：首包使用 motor_id（例如 0x100 / 0x200），后续包使用 motor_id + 1（例如 0x101 / 0x201），
 * 协议末尾包含校验码 CAN_CHECK_CODE（0x6B）。
 */

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
static uint8_t last_cmd[16]; /**< 保存最后一次发送的命令数据（用于重发/调试） */
static uint8_t last_cmd_len = 0; /**< last_cmd 有效长度 */
static StepperMotorID_e last_motor_id = 0; /**< 最后发送的电机 ID */
static uint8_t is_multi_packet = 0; /**< 是否为多帧发送标志 */



/**
 * @brief 通过 CAN 发送步进电机命令（支持单帧与最多两帧发送）
 *
 * @param motor_id 目标电机帧 ID（使用扩展帧，如 0x100/0x200）
 * @param cmd 指向要发送的命令字节数组
 * @param len 命令长度（最大 16 字节）
 * @return true: 收到响应（或发送完成并等到信号量），false: 超时或参数错误
 *
 * @note 协议说明：
 * - 单帧：len <= 8，直接以 motor_id 作为 ExtId 发送完整数据。
 * - 多帧：len > 8，第一帧发送前 8 字节，ExtId = motor_id；第二帧首字节为命令码（cmd[0]），后续为 cmd[8..]，ExtId = motor_id + 1。
 * - 多帧第二包会把命令码重复放在 D0，以便接收端区分包类型（遵循已有设备协议）。
 */
bool StepperCAN_SendCommand(StepperMotorID_e motor_id, uint8_t *cmd, uint8_t len)
{
    if (cmd == NULL || len == 0 || len > 16) {
        return false;
    }

    last_motor_id = motor_id;
    last_cmd_len = len;
    memcpy(last_cmd, cmd, len);
    is_multi_packet = (len > 8) ? 1 : 0;

    uint8_t retries = CAN_MAX_RETRIES;
    bool response_received = false;

    while (retries-- && !response_received) {
        while (osSemaphoreWait(can_cmd_semHandle, 0) == osOK) {}

        if (len <= 8) {
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
        } else {
            // 第一帧
            CAN_TxFrameTypeDef tx_frame1 = {
                .hcan = &hcan1,
                .header = {
                    .ExtId = motor_id,
                    .IDE = CAN_ID_EXT,
                    .RTR = CAN_RTR_DATA,
                    .DLC = 8
                }
            };
            memcpy(tx_frame1.Data, cmd, 8);
            USER_CAN_TxMessage(&tx_frame1);
            osDelay(CAN_PACKET_DELAY_MS);

            // 第二帧（将命令码放在 D0，后面是剩余数据）
            uint8_t second_len = len - 8 + 1; // 例如 12-8+1 = 5
            CAN_TxFrameTypeDef tx_frame2 = {
                .hcan = &hcan1,
                .header = {
                    .ExtId = motor_id + 1,
                    .IDE = CAN_ID_EXT,
                    .RTR = CAN_RTR_DATA,
                    .DLC = second_len
                }
            };
            tx_frame2.Data[0] = cmd[0]; // 把命令码（如 0xFD）放在第二帧的 D0
            memcpy(&tx_frame2.Data[1], cmd + 8, len - 8); // 拷贝剩余数据（包括校验码 0x6B）
            USER_CAN_TxMessage(&tx_frame2);
        }

        if (osSemaphoreWait(can_cmd_semHandle, CAN_SEM_WAIT_TIMEOUT) == osOK) {
            response_received = true;
        } else {
            osDelay(CAN_RETRY_DELAY_MS);
        }
    }

    return response_received;
}
/*---------------------------------------------电机基本控制函数---------------------------------------------------------*/
/* Motor Control Functions */
/**
 * @brief 发送使能/失能命令给指定电机
 * @param motor 目标电机信息（包含 frame_id）
 * @param enable true 表示使能，false 表示失能
 */
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

/**
 * @brief 设置电机速度（开环速度控制命令）
 * @param motor 目标电机信息
 * @param speed 有符号速度值，正为正向，负为反向
 * @param accel 加速度/斜率（单位由下位机协议定义）
 */
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

/**
 * @brief 发送定位命令（按角度）
 * @param motor 目标电机信息（用于角度->脉冲转换）
 * @param angle 目标角度（单位：度）
 * @param speed 目标速度（协议定义单位）
 * @param accel 加速度
 * @param abs_mode 0 表示相对定位，1 表示绝对定位（依下位机协议）
 */
void StepperCAN_SetPositionAngle(StepperMotor_Info_t *motor, float angle, uint16_t speed, uint8_t accel, uint8_t abs_mode)
{
    uint8_t direction = (angle >= 0) ? 0x01 : 0x00;
    float abs_angle = (angle >= 0) ? angle : -angle;
    float pulse_per_deg = (float)motor->subdiv / motor->step_angle;
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

/**
 * @brief 紧急停止命令
 * @param motor_id 目标电机 ID
 */
void StepperCAN_EmergencyStop(StepperMotorID_e motor_id)
{
    uint8_t cmd[4] = {0xFE, 0x98, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

/**
 * @brief 设置单圈零点并可选择是否存储
 * @param motor_id 目标电机 ID
 * @param store_flag 非 0 表示保存零点
 */
void StepperCAN_SetSingleZero(StepperMotorID_e motor_id, uint8_t store_flag)
{
    uint8_t cmd[4] = {0x93, 0x88, store_flag ? 0x01 : 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

/**
 * @brief 触发回零
 * @param motor_id 目标电机 ID
 * @param mode 回零模式，参见 StepperHomeMode_e
 */
void StepperCAN_TriggerHome(StepperMotorID_e motor_id, StepperHomeMode_e mode)
{
    uint8_t cmd[4] = {0x9A, (uint8_t)mode, 0x00, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}

/**
 * @brief 请求读取电机系统状态
 * @param motor_id 目标电机 ID
 */
void StepperCAN_ReadSystemStatus(StepperMotorID_e motor_id)
{
    uint8_t cmd[3] = {0x43, 0x7A, CAN_CHECK_CODE};
    StepperCAN_SendCommand(motor_id, cmd, sizeof(cmd));
}
