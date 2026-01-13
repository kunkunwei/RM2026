#ifndef __CAN_TASK__
#define __CAN_TASK__

#include "bsp_can.h"

// CAN 发送管理器配置
#define CAN_QUEUE_SIZE 64 // 发送队列大小

// CAN 发送队列结构体
typedef struct {
    CAN_TxFrameTypeDef buffer[CAN_QUEUE_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} CAN_Queue_t;

// CAN 管理器结构体 (包含双路CAN)
typedef struct {
    CAN_Queue_t can1_queue;
    CAN_Queue_t can2_queue;
} CAN_TxManager_t;

// API
void CAN_Manager_Init(void);
void CAN_Manager_Update(void);
uint8_t CAN_Manager_Add(CAN_TxFrameTypeDef *frame);

void Can_Task(void const * argument);

#endif // !__CAN_TASK__
