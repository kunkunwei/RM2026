#ifndef __USER_TASK__
#define __USER_TASK__



/**
 * @brief 云台发送给底盘的控制命令
 * @note 使用两帧CAN数据传输
 */
typedef struct __attribute__((packed))
{
    // 第一帧数据（基本控制）
    uint8_t chassis_mode_cmd;    // 底盘模式命令
    uint8_t spinning_cmd;        // 小陀螺开关
    uint8_t jump_cmd;            // 跳跃命令
    uint8_t reserved1;           // 预留

    float gimbal_yaw_angle;      // 云台YAW角度 (rad)
    float target_speed_x;        // 前进速度 (m/s)
    float target_speed_w_z;      // 旋转速度 (rad/s)

    // 第二帧数据（附加控制）
    float target_length;         // 目标腿长 (m)
    float roll_angle;           // 目标Roll角度 (rad)
    uint8_t reserved2[8];        // 预留扩展
} gimbal_to_chassis_data_t;

/**
 * @brief 底盘发送给云台的反馈数据
 * @note 使用一帧CAN数据传输（8字节）
 */
typedef struct __attribute__((packed))
{
    uint8_t chassis_mode_current; // 当前底盘模式
    uint8_t spinning_state;       // 小陀螺状态
    uint8_t jump_state;           // 跳跃状态
    uint8_t chassis_online;       // 在线状态

    float chassis_yaw_angle;      // 底盘YAW角度 (rad)
    float current_speed_x;        // 前进速度 (m/s)
    float current_speed_w_z;      // 旋转速度 (rad/s)
    float chassis_yaw_rate;       // 底盘YAW角速度 (rad/s)
} chassis_to_gimbal_data_t;
/**
 * @brief 通信数据管理结构体
 */
typedef struct
{
    gimbal_to_chassis_data_t gimbal_cmd;       // 云台发送的命令
    chassis_to_gimbal_data_t chassis_feedback; // 底盘反馈的数据

    TickType_t last_chassis_update_time; // 上次收到底盘数据的时间
    uint8_t chassis_online_flag;         // 底盘在线标志
    uint8_t communication_lost_flag;     // 通信丢失标志

    // 统计信息
    uint32_t send_count;    // 发送次数
    uint32_t receive_count; // 接收次数
    uint32_t error_count;   // 错误次数
} gimbal_chassis_comm_t;

extern gimbal_chassis_comm_t gimbal_chassis_comm; // 云台与底盘通信结构体
void User_Task(void const * argument);

void chassis_parse_control_frame1(gimbal_to_chassis_data_t *cmd, uint8_t data[8]);
void chassis_parse_control_frame2(gimbal_to_chassis_data_t *cmd, uint8_t data[8]);

#endif 
