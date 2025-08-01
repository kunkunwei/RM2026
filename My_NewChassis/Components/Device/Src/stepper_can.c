//
// Created by kun on 25-7-27.
//

#include "stepper_can.h"

#include "bsp_can.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"

// 发送CAN命令
// frame_id: CAN帧ID
// cmd: 命令数据
// len: 命令长度
// 该函数将命令数据打包成CAN帧并发送
void StepperCAN_SendCommand(uint8_t frame_id, uint8_t *cmd, uint8_t len) {
    CAN_TxFrameTypeDef tx_frame;
    tx_frame.hcan =&hcan1; // 指定CAN句柄
    tx_frame.header.StdId = frame_id;
    tx_frame.header.IDE=CAN_ID_EXT,
    tx_frame.header.DLC = len;
    tx_frame.header.RTR=CAN_RTR_DATA,
    // tx_frame.header.DLC=8,
    memcpy(tx_frame.Data, cmd, len);
    USER_CAN_TxMessage(&tx_frame);
}
void tset_can()
{
    uint8_t cmd[6] = {0x01, 0xF6, 0x12,0xFF,0x00, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(0x00, cmd, 6);
}
// 触发编码器校准
void StepperCAN_TriggerEncoderCalibrate(uint8_t frame_id) {
    uint8_t cmd[3] = {0x06, 0x45, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}

// 设置当前位置为零点
void StepperCAN_SetZero(uint8_t frame_id) {
    uint8_t cmd[3] = {0x0A, 0x6D, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}

// 读取参数命令
void StepperCAN_ReadEncoder(uint8_t frame_id) {
    uint8_t cmd[2] = {0x30, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}
void StepperCAN_ReadPulseCount(uint8_t frame_id) {
    uint8_t cmd[2] = {0x33, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}

void StepperCAN_ReadPosition(uint8_t frame_id) {
    uint8_t cmd[2] = {0x36, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}
void StepperMotor_UpdatePosition(StepperMotor_Info_t *motor, uint8_t *rx_data)
{
    // 后面4字节为位置
    if (!motor || !rx_data) return;
    if (rx_data[0] == motor->frame_id) {
        motor->position = (int32_t)((rx_data[1] << 24) | (rx_data[2] << 16) | (rx_data[3] << 8) | rx_data[4]);
         motor->angle = (motor->position * 360.0f) / 65536.0f;
    }
}
void StepperCAN_ReadPositionError(uint8_t frame_id) {
    uint8_t cmd[2] = {0x39, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}
void StepperCAN_ReadEnableState(uint8_t frame_id) {
    uint8_t cmd[2] = {0x3A, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}
void StepperCAN_ReadStallFlag(uint8_t frame_id) {
    uint8_t cmd[2] = {0x3E, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}
void StepperCAN_ReadZeroFlag(uint8_t frame_id) {
    uint8_t cmd[2] = {0x3F, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 2);
}


// 修改参数命令
void StepperCAN_SetSubdivision(uint8_t frame_id, uint8_t subdiv) {
    uint8_t cmd[3] = {0x84, subdiv, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}
void StepperCAN_SetSerialAddr(uint8_t frame_id, uint8_t addr) {
    uint8_t cmd[3] = {0xAE, addr, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}

// 运动控制命令
void StepperCAN_Enable(uint8_t frame_id, bool enable) {
    uint8_t cmd[3] = {0xF3, enable ? 0x01 : 0x00, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}
void StepperCAN_SetSpeed(uint8_t frame_id, uint16_t speed, uint8_t direction, uint8_t accel) {
    uint8_t cmd[5] = {0xF6, (direction << 4) | ((speed >> 8) & 0x0F), speed & 0xFF, accel, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 5);
}
void StepperCAN_StoreSpeedParam(uint8_t frame_id, bool store) {
    uint8_t param = store ? 0xC8 : 0xCA;
    uint8_t cmd[3] = {0xFF, param, STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 3);
}
void StepperCAN_Stop(uint8_t frame_id)
{
    uint8_t cmd[8] = {0xFD, 0x12, 0xFF, 0xFF, 0x00, 0x00,0x00,STEPPER_CAN_CHECK_CODE};
    StepperCAN_SendCommand(frame_id, cmd, 8);
}
void StepperCAN_RelativeMove(uint8_t frame_id, uint16_t speed, uint8_t direction, uint8_t accel, uint32_t pulse) {
    uint8_t cmd[8] = {
        0xFD,
        (direction << 4) | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        accel,
        (pulse >> 16) & 0xFF,
        (pulse >> 8) & 0xFF,
        pulse & 0xFF,
        STEPPER_CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(frame_id, cmd, 8);
}
void StepperCAN_RelativeMoveAngle(StepperMotor_Info_t *motor, uint16_t speed,  uint8_t accel, float angle) {
    if (!motor) return;
    uint8_t direction = (angle >= 0) ? 0 : 1;
    uint32_t pulse = (uint32_t)(angle / (motor->step_angle / motor->subdiv));
    uint8_t cmd[8] = {
        0xFD,
        (direction << 4) | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        accel,
        (pulse >> 16) & 0xFF,
        (pulse >> 8) & 0xFF,
        pulse & 0xFF,
        STEPPER_CAN_CHECK_CODE
    };
    StepperCAN_SendCommand(motor->frame_id, cmd, 8);
}

