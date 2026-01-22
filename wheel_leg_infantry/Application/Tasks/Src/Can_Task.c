
#include "can.h"
#include "main.h"
#include "can_task.h"
#include "Chassis_Task.h"
#include "usart.h"
#include "User_Task.h"
#include "vofa.h"

// #define DEBUG
// #define LEG_DEBUG

/* CAN 管理器配置 */
// #define JOINT_SEND_INTERVAL_MS  2   // 关节电机发送间隔: 2ms (高响应)
// #define WHEEL_SEND_INTERVAL_MS  8  // 轮子电机发送间隔: 10ms (低要求)



/* CAN Manager Instance */
CAN_TxManager_t can_manager;

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID,float Postion, float Velocity, float KP, float KD, float Torque);
static void LK9025_Motor_CAN_Send(int16_t right, int16_t left);
static void Damiao_Motor_Enable(uint8_t Motor_ID);




const chassis_move_t* local_chassis ;



// #define LEG_DEBUG
void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
  // extern CAN_TxFrameTypeDef JointTxFrame[4]; // Removed redundant extern
  // const chassis_move_t* local_chassis = get_chassis_control_point();
    local_chassis = get_chassis_control_point();
	CAN_Manager_Init();
    // static float time_send_joint = 0.0f;
    // static float time_send_wheel = 0.0f;

  osDelay(1000);
  Damiao_Motor_Enable(0);
  osDelay(5);
  Damiao_Motor_Enable(1);
  osDelay(5);
  Damiao_Motor_Enable(2);
  osDelay(5);
  Damiao_Motor_Enable(3);
  osDelay(5);

    // tor[0]=-1.0f;
    // tor[1]=1.0f;

  // static uint8_t comm_counter = 0;
	// float angle_set[4] = 0.0f;
  for(;;)
  {
  	// current_time=DWT_GetTimeline_ms();

    // Attempt to drain queue at start of loop
    // CAN_Manager_Update();


    // 关节电机控制 (每2ms发送一次)
    // if ((current_time - time_send_joint) >= JOINT_SEND_INTERVAL_MS) {
        #ifndef LEG_DEBUG
          Damiao_Motor_CAN_Send(3,0,0,0,local_chassis->right_leg.mit_kd,local_chassis->right_leg.front_joint.tor_set);
          Damiao_Motor_CAN_Send(2,0,0,0,local_chassis->right_leg.mit_kd,local_chassis->right_leg.back_joint.tor_set);
      osDelay(1);
          Damiao_Motor_CAN_Send(0,0,0,0,local_chassis->left_leg.mit_kd,local_chassis->left_leg.front_joint.tor_set);
          Damiao_Motor_CAN_Send(1,0,0,0,local_chassis->left_leg.mit_kd,local_chassis->left_leg.back_joint.tor_set);
      osDelay(1);
        #else
      if (switch_is_mid(local_chassis->chassis_RC->rc.s[0]))
      {
          Damiao_Motor_CAN_Send(0,0,0,0,0.4, -2);
          Damiao_Motor_CAN_Send(1,0,0,0,0.4, 2);
          osDelay(1);
          Damiao_Motor_CAN_Send(2,0,0,0,0.4, -2);
          Damiao_Motor_CAN_Send(3,0,0,0,0.4, 2);
          osDelay(1);
          // Damiao_Motor_CAN_Send(0,0,0,0,0.4, local_chassis->jump_state.jump_comtorque[0]);
          // Damiao_Motor_CAN_Send(1,0,0,0,0.4, local_chassis->jump_state.jump_comtorque[1]);
          // osDelay(1);
          // Damiao_Motor_CAN_Send(2,0,0,0,0.4, local_chassis->jump_state.jump_comtorque[2]);
          // Damiao_Motor_CAN_Send(3,0,0,0,0.4, local_chassis->jump_state.jump_comtorque[3]);
          // osDelay(1);
      }
      else
      {
          Damiao_Motor_CAN_Send(0,0,0,0,0,0);
          Damiao_Motor_CAN_Send(1,0,0,0,0,0);
          osDelay(1);
          Damiao_Motor_CAN_Send(2,0,0,0,0,0);
          Damiao_Motor_CAN_Send(3,0,0,0,0,0);
          osDelay(1);
      }
      //     Damiao_Motor_CAN_Send(0,0,0,0,0,0);
      //     Damiao_Motor_CAN_Send(1,0,0,0,0,0);
      // osDelay(1);
      //     Damiao_Motor_CAN_Send(2,0,0,0,0,0);
      //     Damiao_Motor_CAN_Send(3,0,0,0,0,0);
      // osDelay(1);
        #endif
        // time_send_joint = current_time;
    // }

    // 轮子电机控制 (每8ms发送一次)
    // if ((current_time - time_send_wheel) >= WHEEL_SEND_INTERVAL_MS) {
        #ifndef LEG_DEBUG
          LK9025_Motor_CAN_Send(local_chassis-> right_leg.wheel_motor.give_current,local_chassis->left_leg.wheel_motor.give_current);
        #else
          // LK9025_Motor_CAN_Send(1000,1000);
          LK9025_Motor_CAN_Send(0,0);
        #endif
        // time_send_wheel = current_time;
    // }


    osDelay(1);
  }
  /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID,float Postion, float Velocity, float KP, float KD, float Torque){
    static uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
    //uint8_t Motor_ID = ID-1;
    Postion_Tmp  =  float_to_uint(Postion,-12.5f,12.5,16) ;
    Velocity_Tmp =  float_to_uint(Velocity,-45,45,12);
    KP_Tmp = float_to_uint(KP,0,500,12);
    KD_Tmp = float_to_uint(KD,0,5,12);
    Torque_Tmp = float_to_uint(Torque,-20,20,12);
	JointTxFrame[Motor_ID].Data[0] = (uint8_t)(Postion_Tmp>>8);
	JointTxFrame[Motor_ID].Data[1] = (uint8_t)(Postion_Tmp);
	JointTxFrame[Motor_ID].Data[2] = (uint8_t)(Velocity_Tmp>>4);
	JointTxFrame[Motor_ID].Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
	JointTxFrame[Motor_ID].Data[4] = (uint8_t)(KP_Tmp);
	JointTxFrame[Motor_ID].Data[5] = (uint8_t)(KD_Tmp>>4);
	JointTxFrame[Motor_ID].Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
	JointTxFrame[Motor_ID].Data[7] = (uint8_t)(Torque_Tmp);
   USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
   // CAN_Manager_Add(&JointTxFrame[Motor_ID]);
}
static void Damiao_Motor_Enable(uint8_t Motor_ID){
    //uint8_t Motor_ID = ID-1;

	JointTxFrame[Motor_ID].Data[0] = 0xFF;
	JointTxFrame[Motor_ID].Data[1] = 0xFF;
	JointTxFrame[Motor_ID].Data[2] = 0xFF;
	JointTxFrame[Motor_ID].Data[3] = 0xFF;
	JointTxFrame[Motor_ID].Data[4] = 0xFF;
	JointTxFrame[Motor_ID].Data[5] = 0xFF;
	JointTxFrame[Motor_ID].Data[6] = 0xFF;
	JointTxFrame[Motor_ID].Data[7] = 0xFC;

  USER_CAN_TxMessage(&JointTxFrame[Motor_ID]);
  // CAN_Manager_Add(&JointTxFrame[Motor_ID]);
}
static void LK9025_Motor_CAN_Send(int16_t right, int16_t left){

  RMD_L9025_ALL_TxFrame.Data[0] = right;
  RMD_L9025_ALL_TxFrame.Data[1] = (right >> 8);
  RMD_L9025_ALL_TxFrame.Data[2] = left;
  RMD_L9025_ALL_TxFrame.Data[3] = (left >> 8);
  RMD_L9025_ALL_TxFrame.Data[4] = 0;
  RMD_L9025_ALL_TxFrame.Data[5] = 0;
  RMD_L9025_ALL_TxFrame.Data[6] = 0;
  RMD_L9025_ALL_TxFrame.Data[7] = 0;

  USER_CAN_TxMessage(&RMD_L9025_ALL_TxFrame);
  // CAN_Manager_Add(&RMD_L9025_ALL_TxFrame);
}
/**
 * @brief 检查CAN发送邮箱是否有空闲空间
 * @return 1: 有空闲邮箱可发送, 0: 邮箱已满
 */
/*
static inline uint8_t CAN_Mailbox_Available(void)
{
	extern CAN_HandleTypeDef hcan1;
	extern CAN_HandleTypeDef hcan2;

	// 检查CAN1和CAN2的发送邮箱，任意一个有空闲即可
	// HAL_CAN_GetTxMailboxesFreeLevel 返回空闲邮箱数量 (0-3)
	uint32_t can1_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	// uint32_t can2_free = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);

	// 只要有至少1个邮箱空闲，就可以发送
	// return (can1_free > 0 || can2_free > 0);
	return can1_free > 0;
}
*/
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
