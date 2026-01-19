#ifndef __CTL_CHASSIS_H
#define __CTL_CHASSIS_H

#include "main.h"
#include "Chassis_Task.h"

/* 通信协议定义 */
#define UART_FRAME_HEADER1     0xAA
#define UART_FRAME_HEADER2     0x55
#define FRAME_SIZE             16      // 2帧头 + 12数据 + 2CRC

/* 全部使用int16压缩，精度0.001 */

/* 云台->底盘控制帧（12字节） */
#pragma pack(push, 1)
typedef struct {
    uint16_t ctrl_flags;       // bit0-3:模式, bit4:小陀螺, bit5:跳跃
    
    // 全部int16压缩 (*1000)
    int16_t gimbal_yaw;        // 云台YAW角度 *1000 (rad)
    int16_t yaw_rate;          // 云台YAW角速度 *1000 (rad/s)
    int16_t target_speed_x;    // 前进速度 *1000 (m/s)
    int16_t target_speed_wz;   // 旋转速度 *1000 (rad/s)
    int16_t target_length;     // 目标腿长 *1000 (m)
    int16_t roll_angle;        // Roll角度 *1000 (rad)
} gimbal_ctrl_frame_t;
#pragma pack(pop)

/* 底盘->云台反馈帧（12字节） */
#pragma pack(push, 1)
typedef struct {
    uint16_t status_flags;     // bit0-3:模式, bit4:小陀螺, bit5:跳跃状态, bit6:在线
    
    // 全部int16压缩 (*1000)
    int16_t chassis_yaw;       // 底盘YAW角度 *1000 (rad)
    int16_t chassis_yaw_rate;  // 底盘YAW角速度 *1000 (rad/s)
    int16_t current_speed_x;   // 前进速度 *1000 (m/s)
    int16_t current_speed_wz;  // 旋转速度 *1000 (rad/s)
    int16_t current_length;    // 当前腿长 *1000 (m)

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
    gimbal_ctrl_frame_t ctrl_cmd;      // 当前控制命令
    chassis_feedback_frame_t feedback; // 当前反馈数据
    
    float last_rx_time;       // 最后接收时间(ms)
    float last_tx_time;       // 最后发送时间(ms)
    uint8_t comm_ok;          // 通信正常标志
    uint32_t rx_count;        // 接收计数
    uint32_t tx_count;        // 发送计数
} chassis_comm_t;

extern chassis_comm_t chassis_comm;

/* 函数声明 */
void chassis_comm_init(void);
void chassis_send_feedback(const chassis_move_t* chassis);
void chassis_parse_ctrl_cmd(uint8_t *data);
uint8_t chassis_check_comm_status(float current_time);
gimbal_ctrl_frame_t* chassis_get_ctrl_cmd(void);

#endif