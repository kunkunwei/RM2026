#ifndef __ROS_TASK__H__
#define __ROS_TASK__H__

#include "cmsis_os.h"
#include <stdint.h>

#define ROS_PACKET_HEADER 0x42
#define ROS_MAX_DATA_LEN  32
//
// typedef struct {
//     float yaw;      // 通道1：云台yaw角度
//     float pitch;    // 通道2：云台pitch角度
//     uint32_t state; // 通道3：状态机
// } RosParsedData_t;
//
// // 解析状态机
// typedef enum {
//     STATE_WAIT_HEADER,
//     STATE_WAIT_ID,
//     STATE_WAIT_LEN,
//     STATE_WAIT_DATA,
//     STATE_WAIT_CHECKSUM
// } ParserState_t;
//
// // 解析后数据队列
// extern osMessageQId ros_data_queueHandle;

void Ros_Task(void const * argument);

#endif