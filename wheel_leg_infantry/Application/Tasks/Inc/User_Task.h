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
typedef struct {
    // 接收到的控制命令
    gimbal_to_chassis_data_t current_cmd;

    // 通信状态
    float last_frame1_time;  // 第一帧最后接收时间
    float last_frame2_time;  // 第二帧最后接收时间
    float last_valid_time;   // 最后一次有效通信时间

    // 状态标志
    uint8_t frame1_received : 1;    // 第一帧已收到
    uint8_t frame2_received : 1;    // 第二帧已收到
    uint8_t communication_ok : 1;   // 通信正常
    uint8_t safe_mode_active : 1;   // 安全模式激活
    uint8_t reserved : 4;

    // 统计
    uint32_t timeout_count;     // 超时次数
    uint32_t recovery_count;    // 恢复次数
} chassis_comm_manager_t;
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
    // chassis_comm_manager_t gimbal_cmd;       // 云台发送的命令
    chassis_to_gimbal_data_t chassis_feedback; // 底盘反馈的数据

    TickType_t last_chassis_update_time; // 上次收到底盘数据的时间

    // 通信状态
    float last_frame1_time;  // 第一帧最后接收时间
    float last_frame2_time;  // 第二帧最后接收时间
    float last_valid_time;   // 最后一次有效通信时间

    // 状态标志
    uint8_t frame1_received : 1;    // 第一帧已收到
    uint8_t frame2_received : 1;    // 第二帧已收到
    uint8_t comm_ok : 1;        // 通信正常标志
    uint8_t safe_mode : 1;      // 安全模式激活
    // uint8_t reserved : 4;

    // 统计
    uint32_t timeout_count;     // 超时次数
    uint32_t recovery_count;    // 恢复次数
} gimbal_chassis_comm_t;

extern gimbal_chassis_comm_t gimbal_chassis_comm; // 云台与底盘通信结构体
void User_Task(void const * argument);

void chassis_parse_control_frame1(gimbal_to_chassis_data_t *cmd, uint8_t data[8]);
void chassis_parse_control_frame2(gimbal_to_chassis_data_t *cmd, uint8_t data[8]);
const gimbal_chassis_comm_t* get_gimbal_chassis_comm_point(void);
#endif 
