/* CAN Manager Implementations */

/**
 * @brief Helper to update mailbox usage status
 */
static void Update_Mailbox_State(CAN_HandleTypeDef *hcan, uint32_t *ids, uint32_t *ides) {
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0)) ids[0] = 0xFFFFFFFF; // Mark free
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX1)) ids[1] = 0xFFFFFFFF;
    if (!HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX2)) ids[2] = 0xFFFFFFFF;
}

/**
 * @brief Helper to check if ID is pending in mailboxes
 */
static uint8_t Is_ID_Pending(uint32_t stdId, uint32_t ide, uint32_t *ids, uint32_t *ides) {
    for (int i = 0; i < 3; i++) {
        if (ids[i] != 0xFFFFFFFF && ids[i] == stdId && ides[i] == ide) {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Init Mailbox trackers
 */
static void Init_Mailbox_State(void) {
    for(int i=0; i<3; i++) {
        can_manager.can1_mb_ids[i] = 0xFFFFFFFF;
        can_manager.can2_mb_ids[i] = 0xFFFFFFFF;
    }
    can_manager.can1_last_id = 0xFFFFFFFF;
    can_manager.can2_last_id = 0xFFFFFFFF;
}

/**
 * @brief Initialize CAN Manager
 */
void CAN_Manager_Init(void) {
    memset(&can_manager, 0, sizeof(can_manager));
    Init_Mailbox_State();
}

/**
 * @brief Helper to force interleaving by swapping queue head if it matches last sent ID
 */
static void Try_Interleave_Queue(CAN_Queue_t *q, uint32_t last_id) {
    if (q->count < 2) return; // Need at least 2 items to swap

    uint16_t head_idx = q->head;
    // If head ID matches last sent ID
    if (q->buffer[head_idx].header.StdId == last_id) {

        uint16_t next_idx = (head_idx + 1) % CAN_QUEUE_SIZE;
        // Swap head and next
        CAN_TxFrameTypeDef temp = q->buffer[head_idx];
        q->buffer[head_idx] = q->buffer[next_idx];
        q->buffer[next_idx] = temp;
    }
}

/**
 * @brief Update CAN Manager (Process Queues)
 * Call this function frequently to drain the queues.
 */
void CAN_Manager_Update(void) {
    // 1. Update Hardware State (Check what finished sending)
    Update_Mailbox_State(&hcan1, can_manager.can1_mb_ids, can_manager.can1_mb_ide);
    Update_Mailbox_State(&hcan2, can_manager.can2_mb_ids, can_manager.can2_mb_ide);

    // 2. Process CAN1 Queue
    while (can_manager.can1_queue.count > 0) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
            break; // Hardware buffer full
        }

        // Strategy: Anti-Contiguous Transmission
        // If head matches last sent ID, try to swap with next item to interleave
        Try_Interleave_Queue(&can_manager.can1_queue, can_manager.can1_last_id);

        CAN_TxFrameTypeDef *tx_frame = &can_manager.can1_queue.buffer[can_manager.can1_queue.head];

        // CHECK: Is this ID currently in flight? (Concurrency check)
        if (Is_ID_Pending(tx_frame->header.StdId, tx_frame->header.IDE, can_manager.can1_mb_ids, can_manager.can1_mb_ide)) {
            // Drop redundant frame for concurrency
            can_manager.can1_queue.head = (can_manager.can1_queue.head + 1) % CAN_QUEUE_SIZE;
            can_manager.can1_queue.count--;
            continue;
        }

        uint32_t mailbox_mask;
        // Attempt to send
        if (HAL_CAN_AddTxMessage(tx_frame->hcan, &tx_frame->header, tx_frame->Data, &mailbox_mask) == HAL_OK) {
            // Track the mailbox assignment
            int mb_idx = -1;
            if (mailbox_mask == CAN_TX_MAILBOX0) mb_idx = 0;
            else if (mailbox_mask == CAN_TX_MAILBOX1) mb_idx = 1;
            else if (mailbox_mask == CAN_TX_MAILBOX2) mb_idx = 2;

            if (mb_idx >= 0) {
                can_manager.can1_mb_ids[mb_idx] = tx_frame->header.StdId;
                can_manager.can1_mb_ide[mb_idx] = tx_frame->header.IDE;
            }

            // Record last sent ID
            can_manager.can1_last_id = tx_frame->header.StdId;

            can_manager.can1_queue.head = (can_manager.can1_queue.head + 1) % CAN_QUEUE_SIZE;
            can_manager.can1_queue.count--;
        } else {
            break;
        }
    }

    // 3. Process CAN2 Queue
    while (can_manager.can2_queue.count > 0) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0) {
            break; // Hardware buffer full
        }

        Try_Interleave_Queue(&can_manager.can2_queue, can_manager.can2_last_id);

        CAN_TxFrameTypeDef *tx_frame = &can_manager.can2_queue.buffer[can_manager.can2_queue.head];

        // CHECK: Is this ID currently in flight?
        if (Is_ID_Pending(tx_frame->header.StdId, tx_frame->header.IDE, can_manager.can2_mb_ids, can_manager.can2_mb_ide)) {
             // Drop redundant
            can_manager.can2_queue.head = (can_manager.can2_queue.head + 1) % CAN_QUEUE_SIZE;
            can_manager.can2_queue.count--;
            continue;
        }

        uint32_t mailbox_mask;

        if (HAL_CAN_AddTxMessage(tx_frame->hcan, &tx_frame->header, tx_frame->Data, &mailbox_mask) == HAL_OK) {
             // Track the mailbox assignment
            int mb_idx = -1;
            if (mailbox_mask == CAN_TX_MAILBOX0) mb_idx = 0;
            else if (mailbox_mask == CAN_TX_MAILBOX1) mb_idx = 1;
            else if (mailbox_mask == CAN_TX_MAILBOX2) mb_idx = 2;

            if (mb_idx >= 0) {
                can_manager.can2_mb_ids[mb_idx] = tx_frame->header.StdId;
                can_manager.can2_mb_ide[mb_idx] = tx_frame->header.IDE;
            }

            can_manager.can2_last_id = tx_frame->header.StdId;

            can_manager.can2_queue.head = (can_manager.can2_queue.head + 1) % CAN_QUEUE_SIZE;
            can_manager.can2_queue.count--;
        } else {
            break;
        }
    }
}

/**
 * @brief Add frame to CAN Manager Queue
 * 核心逻辑：防止队列中连续存在或堆积相同的ID。
 * 如果队列中已经有相同ID的帧，直接覆盖旧数据，不增加队列长度。
 */
uint8_t CAN_Manager_Add(CAN_TxFrameTypeDef *frame) {
    CAN_Queue_t *q = NULL;

    if (frame->hcan == &hcan1) {
        q = &can_manager.can1_queue;
    } else if (frame->hcan == &hcan2) {
        q = &can_manager.can2_queue;
    } else {
        return 0; // Unknown bus
    }

    // 1. 检查队列中是否已有相同ID的帧 (防止同一ID堆积爆满)
    for (uint16_t i = 0; i < q->count; i++) {
        uint16_t idx = (q->head + i) % CAN_QUEUE_SIZE;
        // 比较标准ID (如果是扩展帧需比较ExtId，这里假设都是标准帧)
        if (q->buffer[idx].header.StdId == frame->header.StdId &&
            q->buffer[idx].header.IDE == frame->header.IDE) {

            // 找到重复ID，覆盖旧数据 (保持最新控制指令)
            q->buffer[idx] = *frame;

            // 尝试触发发送 (如果刚才覆盖的是队头，Update可能会发出去)
            CAN_Manager_Update();
            return 1;
        }
    }

    // 2. 队列已满处理
    if (q->count >= CAN_QUEUE_SIZE) {
        return 0; // 丢弃新数据，或考虑覆盖队尾
    }

    // 3. 入队 (FIFO)
    q->buffer[q->tail] = *frame; // Struct copy
    q->tail = (q->tail + 1) % CAN_QUEUE_SIZE;
    q->count++;

    // Try to send immediately
    CAN_Manager_Update();

    return 1;
}