#include "ctl_chassis.h"
#include "bsp_dwt.h"
#include "usart.h"
#include "vofa.h"

// 定义速度档位（可根据实际需要调整）
#define SPEED_NORMAL_FORWARD   300   // 正常前进速度
#define SPEED_NORMAL_BACKWARD  -300  // 正常后退速度
#define SPEED_FAST_FORWARD     600   // 快速前进速度（Shift按下）
#define SPEED_FAST_BACKWARD    -600  // 快速后退速度（Shift按下）
#define SPEED_SLOW_FORWARD     150   // 慢速前进速度（Ctrl按下）
#define SPEED_SLOW_BACKWARD    -150  // 慢速后退速度（Ctrl按下）

#define SPEED_NORMAL_ROTATE    300   // 正常旋转速度
#define SPEED_FAST_ROTATE      600   // 快速旋转速度（Shift按下）
#define SPEED_SLOW_ROTATE      150   // 慢速旋转速度（Ctrl按下）
/* 全局通信结构体（static: 只允许本 TU 内写入，外部通过 gimbal_get_feedback() 只读访问） */
static gimbal_comm_t gimbal_comm = {0};

/* 初始化通信 */
void gimbal_comm_init(void)
{
    gimbal_comm.last_rx_time = DWT_GetTimeline_ms();
    gimbal_comm.last_tx_time = gimbal_comm.last_rx_time;
    gimbal_comm.comm_ok = 0;
    // uart_printf(&huart6, "Gimbal UART6 Comm Init OK\r\n");
}

/* 发送控制帧 - 修正版：移除 *1000 放大，防止 int16 溢出 */
void gimbal_send_ctrl_cmd(const gimbal_t *gimbal)
{
    static uint8_t tx_buffer[FRAME_SIZE];
    static float last_tx_time = 0;

    // 获取当前时间（你需要适配你的获取时间函数，如 DWT_GetTimeline_ms 或 HAL_GetTick）
    // float current_time = DWT_GetTimeline_ms();
    // 假设使用 HAL_GetTick
    uint32_t current_time = HAL_GetTick();

    if (gimbal == NULL)
    {
        return;
    }

    // 30ms发送周期
    if (current_time - last_tx_time < 30) return;

    // 设置帧头
    tx_buffer[0] = UART_FRAME_HEADER1;
    tx_buffer[1] = UART_FRAME_HEADER2;

    // 构建控制标志
    uint16_t ctrl_flags = gimbal->chassis_cmd.chassis_mode_cmd & CTRL_MODE_MASK;
    if (gimbal->chassis_cmd.spinning_cmd) ctrl_flags |= CTRL_SPINNING_MASK;
    if (gimbal->chassis_cmd.jump_cmd) ctrl_flags |= CTRL_JUMP_MASK;

    // 小端格式写入控制标志
    tx_buffer[2] = (uint8_t)(ctrl_flags & 0xFF);
    tx_buffer[3] = (uint8_t)(ctrl_flags >> 8);

    /*
     * 关键修改：直接发送原始数据（int16），不进行 *1000 放大
     * 原理：RC通道值通常在±660左右，如果 *1000 会溢出 int16 (±32767)
     * 底盘端已经同步修改为不除以 1000 (target_speed_x/wz) 或已经使用原始值 (target_length)
     */

    // 写入各个int16数据（小端）
    // 云台Yaw角度：如果是弧度制且范围在±30以内，可以保留 *1000
    // 如果是角度制或很大，也需要取消 *1000
    // 假设是弧度 (rad)，范围 -3.14~3.14， *1000 = -3140~3140，不会溢出
    int16_t yaw = (int16_t)(gimbal->gimbal_pos.yaw_relattive_pos * 1000.0f);
    tx_buffer[4] = (uint8_t)(yaw & 0xFF);
    tx_buffer[5] = (uint8_t)(yaw >> 8);

    // 目标速度 X：直接发送原始值 (RC -660~660)
    int16_t speed_x = (int16_t)(gimbal->chassis_cmd.target_speed_x);
    tx_buffer[6] = (uint8_t)(speed_x & 0xFF);
    tx_buffer[7] = (uint8_t)(speed_x >> 8);

    // 目标速度 Wz：直接发送原始值
    int16_t speed_wz = (int16_t)(gimbal->chassis_cmd.target_speed_w_z);
    tx_buffer[8] = (uint8_t)(speed_wz & 0xFF);
    tx_buffer[9] = (uint8_t)(speed_wz >> 8);

    // 目标腿长：直接发送原始值
    int16_t length = (int16_t)(gimbal->chassis_cmd.target_length);
    tx_buffer[10] = (uint8_t)(length & 0xFF);
    tx_buffer[11] = (uint8_t)(length >> 8);

    // Roll角度：直接发送原始值 (通常也是RC通道值)
    int16_t roll = (int16_t)(gimbal->chassis_cmd.roll_angle);
    tx_buffer[12] = (uint8_t)(roll & 0xFF);
    tx_buffer[13] = (uint8_t)(roll >> 8);

    // 发送 yaw_absolute_pos
    int16_t yaw_absolute_pos = (int16_t)(gimbal->gimbal_pos.yaw_absolute_pos * 1000.0f);
    tx_buffer[14] = (uint8_t)(yaw_absolute_pos & 0xFF);
    tx_buffer[15] = (uint8_t)(yaw_absolute_pos >> 8);

    // 发送 yaw_gyro
    int16_t yaw_gyro = (int16_t)(gimbal->ins_info->yaw_gyro * 1000.0f);
    tx_buffer[16] = (uint8_t)(yaw_gyro & 0xFF);
    tx_buffer[17] = (uint8_t)(yaw_gyro >> 8);

    // 添加CRC并发送
    append_CRC16_check_sum(tx_buffer, FRAME_SIZE);

    // 发送数据 (假设 huart6)
    HAL_UART_Transmit(&huart6, tx_buffer, FRAME_SIZE, 10);
    // 或者是 DMA 发送
    // HAL_UART_Transmit_DMA(&huart6, tx_buffer, FRAME_SIZE);

    last_tx_time = current_time;
}

/* 解析反馈帧 - 直接解析，不复制 */
void gimbal_parse_feedback(uint8_t *data)
{
    // 检查帧头
    if (data[0] != UART_FRAME_HEADER1 || data[1] != UART_FRAME_HEADER2) {
        // uart_printf(&huart1,"FRAME_ERR!\r\n");
        return;
    }
    
    // 检查CRC
    if (!verify_CRC16_check_sum(data, FRAME_SIZE)) {
        // uart_printf(&huart1,"CRC_ERR!\r\n");
        return;
    }
    // uart_printf(&huart1,"CRC_SUCCESS!\r\n");
    // 直接解析数据（避免memcpy）
    // HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    // 注意：数据是小端格式
     gimbal_comm.feedback.status_flags = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
     gimbal_comm.feedback.chassis_yaw = (int16_t)(data[4] | ((int16_t)data[5] << 8));
     gimbal_comm.feedback.current_speed_x = (int16_t)(data[6] | ((int16_t)data[7] << 8));
     gimbal_comm.feedback.current_speed_wz = (int16_t)(data[8] | ((int16_t)data[9] << 8));
     gimbal_comm.feedback.current_length = (int16_t)(data[10] | ((int16_t)data[11] << 8));
     gimbal_comm.feedback.yaw_rate = (int16_t)(data[12] | ((int16_t)data[13] << 8));

    // 更新接收状态
    gimbal_comm.last_rx_time = DWT_GetTimeline_ms();
    gimbal_comm.rx_count++;
    gimbal_comm.comm_ok = 1;
}

/* 检查通信状态 */
uint8_t gimbal_check_comm_status(float current_time)
{
    float dt = current_time - gimbal_comm.last_rx_time;
    uint8_t new_state = (dt < 20.0f) && (gimbal_comm.rx_count > 0);
    
    if (new_state != gimbal_comm.comm_ok) {
        gimbal_comm.comm_ok = new_state;
        
        if (!new_state) {
            // uart_printf(&huart1, "Chassis comm lost, dt=%.1fms\r\n", dt);
        } else {
            // uart_printf(&huart1, "Chassis comm recovered\r\n");
        }
    }
    
    return new_state;
}

/* 获取当前反馈数据（只读） */
const chassis_feedback_frame_t* gimbal_get_feedback(void)
{
    return &gimbal_comm.feedback;
}

/* 获取浮点格式的反馈数据（方便使用） */
void gimbal_get_feedback_float(float *chassis_yaw, float *speed_x, 
                                float *speed_wz, float *length, float *yaw_rate)
{
    chassis_feedback_frame_t *fb = &gimbal_comm.feedback;
    
    if (chassis_yaw) *chassis_yaw = fb->chassis_yaw / 1000.0f;
    if (speed_x) *speed_x = fb->current_speed_x / 1000.0f;
    if (speed_wz) *speed_wz = fb->current_speed_wz / 1000.0f;
    if (length) *length = fb->current_length / 1000.0f;
    if (yaw_rate) *yaw_rate = fb->yaw_rate / 1000.0f;
}
void keyboard_control(gimbal_t *gimbal)
{
    // 前后控制（W/S）
    if(PC_KeyBoard_W && !PC_KeyBoard_S)
    {
        // 前进
        if(PC_KeyBoard_SHIFT)
            gimbal->chassis_cmd.target_speed_x = 600;      // 快速前进
        else if(PC_KeyBoard_CTRL)
            gimbal->chassis_cmd.target_speed_x = 150;      // 慢速前进
        else
            gimbal->chassis_cmd.target_speed_x = 300;      // 正常前进
    }
    else if(PC_KeyBoard_S && !PC_KeyBoard_W)
    {
        // 后退
        if(PC_KeyBoard_SHIFT)
            gimbal->chassis_cmd.target_speed_x = -600;     // 快速后退
        else if(PC_KeyBoard_CTRL)
            gimbal->chassis_cmd.target_speed_x = -150;     // 慢速后退
        else
            gimbal->chassis_cmd.target_speed_x = -300;     // 正常后退
    }

    // 旋转控制（A/D）
    if(PC_KeyBoard_A && !PC_KeyBoard_D)
    {
        // 逆时针旋转
        if(PC_KeyBoard_SHIFT)
            gimbal->chassis_cmd.target_speed_w_z = 600;    // 快速逆时针
        else if(PC_KeyBoard_CTRL)
            gimbal->chassis_cmd.target_speed_w_z = 150;    // 慢速逆时针
        else
            gimbal->chassis_cmd.target_speed_w_z = 300;    // 正常逆时针
    }
    else if(PC_KeyBoard_D && !PC_KeyBoard_A)
    {
        // 顺时针旋转
        if(PC_KeyBoard_SHIFT)
            gimbal->chassis_cmd.target_speed_w_z = -600;   // 快速顺时针
        else if(PC_KeyBoard_CTRL)
            gimbal->chassis_cmd.target_speed_w_z = -150;   // 慢速顺时针
        else
            gimbal->chassis_cmd.target_speed_w_z = -300;   // 正常顺时针
    }
    //跳跃和小陀螺切换键（E/Q）
    // if (PC_KeyBoard_E&&(!PC_KeyBoard_SHIFT))
    // {
    //     gimbal->chassis_cmd.spinning_cmd=true;
    // }
    // else
    // {
    //     gimbal->chassis_cmd.spinning_cmd=false;
    // }
    //
    // if (PC_KeyBoard_Q&&(!gimbal->chassis_cmd.spinning_cmd))
    // gimbal->chassis_cmd.jump_cmd=PC_KeyBoard_Q;
}
/**
 * @brief 根据云台当前模式和遥控器输入，填充底盘控制指令并通过USART6发送
 * @param gimbal 云台控制结构体指针
 */
void gimbal_ctl_chassis_cmd(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;

    /* 根据云台模式映射底盘工作模式 */
    switch (gimbal->gimbal_mod) {
    case GIMBAL_MOD_NO_FORCE:
    case GIMBAL_MOD_SLOW_CALI:
        gimbal->chassis_cmd.chassis_mode_cmd = CHASSIS_FORCE_RAW;
        break;
    case GIMBAL_MOD_NORMAL:
        gimbal->chassis_cmd.chassis_mode_cmd = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        break;
    case GIMBAL_MOD_AutoAim:
    case GIMBAL_MOD_FLOW_CHASSIS:
        gimbal->chassis_cmd.chassis_mode_cmd = CHASSIS_VECTOR_NO_FOLLOW_YAW;
        break;
    case GIMBAL_MOD_CONTROL_BY_PC:
        break;
    default:
        gimbal->chassis_cmd.chassis_mode_cmd = CHASSIS_FORCE_RAW;
        break;
    }

    /* 填充云台位姿数据，供底盘跟随参考 */
    gimbal->chassis_cmd.gimbal_yaw_angle = gimbal->gimbal_pos.yaw_relattive_pos;
    gimbal->chassis_cmd.gimbal_yaw_gyro  = gimbal->ins_info->yaw_gyro;
#ifdef  USE_PC_CONTROL
    /* 填充运动速度指令（直接使用遥控器通道原始值，底盘侧缩放） */

    gimbal->chassis_cmd.target_speed_x   = gimbal->gimbal_PC_RC->rc.ch[RC_LEFT_Y_CH];
    gimbal->chassis_cmd.target_speed_w_z = gimbal->gimbal_PC_RC->rc.ch[RC_LEFT_X_CH];

    gimbal->chassis_cmd.target_length    = gimbal->gimbal_PC_RC->rc.ch[RC_RIGHT_X_CH]||gimbal->gimbal_PC_RC->mouse.z;

    /* 小陀螺模式：右fn_2按住 且处于正常模式时激活 */
    gimbal->chassis_cmd.spinning_cmd =gimbal->spinning_mode_cmd;
    if (gimbal->gimbal_mod==GIMBAL_MOD_NORMAL||gimbal->gimbal_mod==GIMBAL_MOD_FLOW_CHASSIS)
    {
        gimbal->chassis_cmd.jump_cmd=gimbal->gimbal_PC_RC->rc.pause;
    }
    else
    {
        gimbal->chassis_cmd.jump_cmd=false;
    }
    //键盘输入控制
    keyboard_control(gimbal);
#else
#ifdef USE_SBUS_PROTOCOL
    /* 填充运动速度指令（直接使用遥控器通道原始值，底盘侧缩放） */
    gimbal->chassis_cmd.target_speed_x   = gimbal->gimbal_RC->rc.ch[RC_LEFT_Y_CH];
    gimbal->chassis_cmd.target_speed_w_z = gimbal->gimbal_RC->rc.ch[RC_LEFT_X_CH];
    gimbal->chassis_cmd.target_length    = gimbal->gimbal_RC->rc.ch[RC_ROLL_2_CH];

    /* 小陀螺模式：右2拨杆下拨且处于正常模式时激活 */
    gimbal->chassis_cmd.spinning_cmd =
        (gimbal->gimbal_RC->rc.s[RIGHT_2_SWITCH] == 1 &&
         gimbal->gimbal_mod == GIMBAL_MOD_NORMAL);
#else
    /* 填充运动速度指令（直接使用遥控器通道原始值，底盘侧缩放） */
    gimbal->chassis_cmd.target_speed_x   = gimbal->gimbal_RC->rc.ch[RC_LEFT_Y_CH];
    gimbal->chassis_cmd.target_speed_w_z = gimbal->gimbal_RC->rc.ch[RC_LEFT_X_CH];
    gimbal->chassis_cmd.target_length    = gimbal->gimbal_RC->rc.ch[RC_RIGHT_Y_CH];

    /* 小陀螺模式：右2拨杆下拨且处于正常模式时激活 */
    gimbal->chassis_cmd.spinning_cmd =
        (switch_is_down(gimbal->gimbal_RC->rc.s[LEFT_SWITCH]) &&
         gimbal->gimbal_mod == GIMBAL_MOD_NORMAL);
#endif
#endif

    /* 打包串口帧并通过USART6 DMA发送 */
    gimbal_send_ctrl_cmd(gimbal);
}