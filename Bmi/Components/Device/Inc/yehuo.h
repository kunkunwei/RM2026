//
// Created by kun on 25-7-22.
//

#ifndef YEHUO_H
#define YEHUO_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>

#define FIRE_FRAME_HEAD 0x59485A53U  // "YHZS" 低字节优先，实际发送顺序：0x53 0x5A 0x48 0x59
#define FIRE_MAX_DATA_LEN 16
#define FIRE_CHANNEL 0x01           // 通道编号（可设为1~5）

typedef enum {
    FIRE_CMD_SET_PID = 0x10,
    FIRE_CMD_SET_TARGET = 0x11,
    FIRE_CMD_START = 0x12,
    FIRE_CMD_STOP = 0x13,
    FIRE_CMD_RESET = 0x14,
    FIRE_CMD_SET_PERIOD = 0x15,
    FIRE_CMD_REPORT_VALUE = 0x20    // 自定义：上传调试值
} Fire_Command_t;

typedef struct {
    uint32_t head;            // 固定帧头
    uint8_t channel;
    uint8_t length;
    uint8_t cmd;
    uint8_t data[FIRE_MAX_DATA_LEN];
    uint8_t checksum;
} __packed Fire_Frame_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float target;
    uint8_t started;
    uint8_t period_ms;
} PID_Info_t;
#endif //YEHUO_H
