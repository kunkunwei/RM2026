#ifndef __CTL_CHASSIS_H
#define __CTL_CHASSIS_H

#include "main.h"
#include "crc.h"
#include "Gimbal_task.h"

/* 通信协议定义 */
#define UART_FRAME_HEADER1     0xAA
#define UART_FRAME_HEADER2     0x55
#define FRAME_SIZE             20      // 2帧头 + 16数据 + 2CRC

/* 全部使用int16压缩，精度0.001 */

/* 底盘->云台反馈帧（12字节） */
#pragma pack(push, 1)
typedef struct {
    uint16_t status_flags;     // bit0-3:模式, bit4:小陀螺, bit5:跳跃状态, bit6:在线
    int16_t chassis_yaw;       // 底盘YAW角度 *1000 (rad)
    int16_t current_speed_x;   // 前进速度 *1000 (m/s)
    int16_t current_speed_wz;  // 旋转速度 *1000 (rad/s)
    int16_t current_length;    // 当前腿长 *1000 (m)
    int16_t yaw_rate;          // YAW角速度 *1000 (rad/s)
} chassis_feedback_frame_t;
#pragma pack(pop)

/* 标志位掩码 */
#define CTRL_MODE_MASK         0x000F
#define CTRL_SPINNING_MASK     0x0010
#define CTRL_JUMP_MASK         0x0020

#define STATUS_MODE_MASK       0x000F
#define STATUS_SPINNING_MASK   0x0010
#define STATUS_JUMP_MASK       0x0020
#define STATUS_ONLINE_MASK     0x0040

/* 简化通信管理结构体 */
typedef struct {
    chassis_feedback_frame_t feedback; // 当前反馈数据
    float last_rx_time;       // 最后接收时间(ms)
    float last_tx_time;       // 最后发送时间(ms)
    uint8_t comm_ok;          // 通信正常标志
    uint32_t rx_count;        // 接收计数
    uint32_t tx_count;        // 发送计数
} gimbal_comm_t;

/* gimbal_comm 已在 ctl_chassis.c 中声明为 static，外部通过 gimbal_get_feedback() 只读访问 */

/* 函数声明 */
void gimbal_comm_init(void);
void gimbal_send_ctrl_cmd(const gimbal_t *gimbal);
void gimbal_parse_feedback(uint8_t *data);
uint8_t gimbal_check_comm_status(float current_time);
const chassis_feedback_frame_t* gimbal_get_feedback(void);
/**
 * @brief 根据云台当前模式和遥控器输入，填充底盘控制指令并通过USART6发送
 * @param gimbal 云台控制结构体指针
 * @note  封装了原先在 Gimbal_task.c 中的发送逻辑，统一归属于底盘通信设备模块
 */
void gimbal_ctl_chassis_cmd(gimbal_t *gimbal);

#endif