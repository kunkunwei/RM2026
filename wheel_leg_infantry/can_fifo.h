// CAN 发送管理器配置
#define CAN_QUEUE_SIZE 128 // 增加发送队列大小以防止突发数据丢失

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

    // 硬件邮箱状态跟踪 (用于防止重复ID并发发送)
    uint32_t can1_mb_ids[3]; // CAN1 邮箱中的 ID
    uint32_t can1_mb_ide[3]; // CAN1 邮箱中的 IDE (Standard/Extended)

    uint32_t can2_mb_ids[3]; // CAN2 邮箱中的 ID
    uint32_t can2_mb_ide[3]; // CAN2 邮箱中的 IDE

    // 上一次发送的ID记录 (用于强制交替发送)
    uint32_t can1_last_id;
    uint32_t can2_last_id;
} CAN_TxManager_t;

// API
void CAN_Manager_Init(void);
void CAN_Manager_Update(void);
uint8_t CAN_Manager_Add(CAN_TxFrameTypeDef *frame);