//
// Created by kun on 25-7-22.
//

#include "yehuo.h"

// 计算简单8位校验和
static uint8_t Fire_CalcChecksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

// 打包并发送 float 调试值（最多发送3个float）
HAL_StatusTypeDef Fire_SendDebugValue(UART_HandleTypeDef *huart, float *val, uint8_t count) {
    if (count > 3) return HAL_ERROR;  // 最大支持3个float
    Fire_Frame_t frame;
    frame.head = FIRE_FRAME_HEAD;
    frame.channel = FIRE_CHANNEL;
    frame.cmd = FIRE_CMD_REPORT_VALUE;
    frame.length = 4 + 1 + 1 + 1 + 4 * count + 1; // 帧头+通道+长度+命令+数据+校验

    memcpy(frame.data, val, 4 * count);
    frame.checksum = Fire_CalcChecksum((uint8_t*)&frame, frame.length - 1);

    return HAL_UART_Transmit(huart, (uint8_t*)&frame, frame.length, 100);
}
HAL_StatusTypeDef Fire_ParseFrame(uint8_t *rx_buf, PID_Info_t *pid) {
    Fire_Frame_t *frame = (Fire_Frame_t *)rx_buf;

    // 帧头校验
    if (frame->head != FIRE_FRAME_HEAD) return HAL_ERROR;
    if (Fire_CalcChecksum((uint8_t*)frame, frame->length - 1) != frame->checksum) return HAL_ERROR;

    switch (frame->cmd) {
    case FIRE_CMD_SET_PID:
        memcpy(&pid->kp, frame->data, 4);
        memcpy(&pid->ki, frame->data + 4, 4);
        memcpy(&pid->kd, frame->data + 8, 4);
        break;
    case FIRE_CMD_SET_TARGET:
        memcpy(&pid->target, frame->data, 4);
        break;
    case FIRE_CMD_START:
        pid->started = 1;
        break;
    case FIRE_CMD_STOP:
        pid->started = 0;
        break;
    case FIRE_CMD_RESET:
        pid->target = 0;
        pid->kp = pid->ki = pid->kd = 0;
        break;
    case FIRE_CMD_SET_PERIOD:
        pid->period_ms = frame->data[0];
        break;
    default:
        return HAL_ERROR;
    }
    return HAL_OK;
}