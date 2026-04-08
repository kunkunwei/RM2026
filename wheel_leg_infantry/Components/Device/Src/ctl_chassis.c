#include "ctl_chassis.h"
#include "bsp_dwt.h"
#include "crc.h"
#include "referee_info.h"
#include "usart.h"
#include "vofa.h"

extern Referee_Info_TypeDef Referee_Info;
/* 全局通信结构体 */
static chassis_comm_t chassis_comm = {0};

/* 初始化通信 */
void chassis_comm_init(void)
{
    chassis_comm.last_rx_time = DWT_GetTimeline_ms();
    chassis_comm.last_tx_time = chassis_comm.last_rx_time;
    chassis_comm.comm_ok = 0;
}

/* 进入安全模式 */
static void enter_safe_mode(void)
{
    // 只清零速度和模式，保持其他参数
    chassis_comm.ctrl_cmd.ctrl_flags = 0; // CHASSIS_FORCE_RAW = 0
    chassis_comm.ctrl_cmd.target_speed_x = 0;
    chassis_comm.ctrl_cmd.target_speed_wz = 0;
}

/* 检查控制命令有效性 */
// 禁用校验，因为上下位机范围不一致
static uint8_t check_ctrl_cmd_valid(gimbal_ctrl_frame_t *cmd)
{

    return 1;
}

/* 发送反馈帧 */
void chassis_send_feedback(const chassis_move_t* chassis)
{
    static uint8_t tx_buffer[FRAME_SIZE];
    float current_time = DWT_GetTimeline_ms();
    
    if (current_time - chassis_comm.last_tx_time < 30.0f) return;

    // 设置帧头
    tx_buffer[0] = UART_FRAME_HEADER1;
    tx_buffer[1] = UART_FRAME_HEADER2;

    // 直接填充数据到缓冲区（避免结构体问题）
    uint16_t status = chassis->chassis_mode & STATUS_MODE_MASK;
    if (chassis->spining_state) status |= STATUS_SPINNING_MASK;
    if (chassis->jump_state.jump_stage) status |= STATUS_JUMP_MASK;
    if (chassis_comm.comm_ok) status |= STATUS_ONLINE_MASK;

    // 小端格式写入
    tx_buffer[2] = (uint8_t)(status & 0xFF);
    tx_buffer[3] = (uint8_t)(status >> 8);

    // 写入各个int16数据（小端）
    int16_t yaw = (int16_t)(chassis->chassis_yaw * 1000.0f);
    tx_buffer[4] = (uint8_t)(yaw & 0xFF);
    tx_buffer[5] = (uint8_t)(yaw >> 8);

    int16_t yaw_rate = (int16_t)(chassis->chassis_imu_gyro[IMU_GYRO_INDEX_YAW] * 1000.0f);
    tx_buffer[6] = (uint8_t)(yaw_rate & 0xFF);
    tx_buffer[7] = (uint8_t)(yaw_rate >> 8);

    int16_t speed_x = (int16_t)(chassis->state_ref.x_dot * 1000.0f);
    tx_buffer[8] = (uint8_t)(speed_x & 0xFF);
    tx_buffer[9] = (uint8_t)(speed_x >> 8);

    int16_t speed_wz = (int16_t)(chassis->wz * 1000.0f);
    tx_buffer[10] = (uint8_t)(speed_wz & 0xFF);
    tx_buffer[11] = (uint8_t)(speed_wz >> 8);

    int16_t length = (int16_t)(chassis->leg_length * 1000.0f);
    tx_buffer[12] = (uint8_t)(length & 0xFF);
    tx_buffer[13] = (uint8_t)(length >> 8);

    // Referee_Info.power_heat.shooter_id1_17mm_cooling_heat
    // Referee_Info.shoot_data.shooter_id
    // Referee_Info.shoot_data.bullet_speed
    // Referee_Info.shoot_data.bullet_freq
    // Referee_Info.robot_status.robot_id
    // Referee_Info.robot_status.robot_level
    // Referee_Info.robot_status.remain_HP
    // 添加CRC并发送
    append_CRC16_check_sum(tx_buffer, FRAME_SIZE);
    // HAL_UART_Transmit_DMA(&huart6, tx_buffer, FRAME_SIZE);
    HAL_UART_Transmit(&huart6, tx_buffer, FRAME_SIZE,100);
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
    chassis_comm.last_tx_time = current_time;
    chassis_comm.tx_count++;
}

/* 解析控制帧 */
void chassis_parse_ctrl_cmd(uint8_t *data)
{

    
    // 检查帧头
    if (data[0] != UART_FRAME_HEADER1 || data[1] != UART_FRAME_HEADER2) {
        // uart_printf(&huart1,"FRAME_ERR! H1=0x%02X H2=0x%02X\r\n", data[0], data[1]);
        return;
    }
    
    // 检查CRC
    if (!verify_CRC16_check_sum(data, FRAME_SIZE)) {
        // 计算并打印CRC用于调试
        // uint16_t calc_crc = get_CRC16_check_sum(data, FRAME_SIZE - 2, 0xffff);
        // uint16_t recv_crc = (data[FRAME_SIZE-1] << 8) | data[FRAME_SIZE-2];
        // uart_printf(&huart1,"CRC_ERR! Calc=0x%04X Recv=0x%04X\r\n", calc_crc, recv_crc);
        return;
    }
    // uart_printf(&huart1,"CRC_SUCCESS!\r\n");
    
    // 真正的解析开始
    // 1. 解析控制标志位（小端格式）
    chassis_comm.ctrl_cmd.ctrl_flags = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

    // 2. 解析云台YAW角度（int16，小端）
    chassis_comm.ctrl_cmd.gimbal_relate_yaw = (int16_t)(data[4] | ((int16_t)data[5] << 8));

    // 3. 解析前进速度（int16，小端）
    chassis_comm.ctrl_cmd.target_speed_x = (int16_t)(data[6] | ((int16_t)data[7] << 8));

    // 4. 解析旋转速度（int16，小端）
    chassis_comm.ctrl_cmd.target_speed_wz = (int16_t)(data[8] | ((int16_t)data[9] << 8));

    // 5. 解析目标腿长（int16，小端）
    chassis_comm.ctrl_cmd.target_length = (int16_t)(data[10] | ((int16_t)data[11] << 8));

    // 6. 解析Roll角度（int16，小端）
    chassis_comm.ctrl_cmd.roll_angle = (int16_t)(data[12] | ((int16_t)data[13] << 8));

    // 7. 解析保留字段（int16，小端）
    chassis_comm.ctrl_cmd.gimbal_yaw_absolute_pos = (int16_t)(data[14] | ((int16_t)data[15] << 8));
    chassis_comm.ctrl_cmd.gimbal_yaw_gyro = (int16_t)(data[16] | ((int16_t)data[17] << 8));

    // // 调试：打印解析结果
    // uart_printf(&huart1, "Parsed: flags=0x%04X yaw=%d spd_x=%d spd_wz=%d len=%d roll=%d reserved=%d\r\n",
    //            chassis_comm.ctrl_cmd.ctrl_flags,
    //            chassis_comm.ctrl_cmd.gimbal_yaw,
    //            chassis_comm.ctrl_cmd.target_speed_x,
    //            chassis_comm.ctrl_cmd.target_speed_wz,
    //            chassis_comm.ctrl_cmd.target_length,
    //            chassis_comm.ctrl_cmd.roll_angle,
    //            chassis_comm.ctrl_cmd.reserved);

    // 数据有效性检查
    if (!check_ctrl_cmd_valid(&chassis_comm.ctrl_cmd)) {
        enter_safe_mode();
        return;
    }

    // 翻转LED指示收到数据
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

    // 更新接收状态
    chassis_comm.last_rx_time = DWT_GetTimeline_ms();
    chassis_comm.rx_count++;
    chassis_comm.comm_ok = 1;
    
    // uart_printf(&huart1, "Rx OK! Count=%lu Time=%.1f\r\n", chassis_comm.rx_count, chassis_comm.last_rx_time);
}

/* 检查通信状态 */
uint8_t chassis_check_comm_status(float current_time)
{
    float dt = current_time - chassis_comm.last_rx_time;
    uint8_t new_state = (dt < 20.0f) && (chassis_comm.rx_count > 0);
    
    if (new_state != chassis_comm.comm_ok) {
        chassis_comm.comm_ok = new_state;
        
        if (!new_state) {
            enter_safe_mode();
        }
    }
    
    return new_state;
}

/* 获取当前控制命令（自动处理断联） */
gimbal_ctrl_frame_t* chassis_get_ctrl_cmd(void)
{
    static gimbal_ctrl_frame_t safe_cmd = {0};
    
    if (chassis_comm.comm_ok) {
        return &chassis_comm.ctrl_cmd;
    } else {
        // 安全模式：零力，保持其他参数
        safe_cmd.ctrl_flags = 0; // CHASSIS_FORCE_RAW = 0
        safe_cmd.target_speed_x = 0;
        safe_cmd.target_speed_wz = 0;
        safe_cmd.gimbal_relate_yaw = chassis_comm.ctrl_cmd.gimbal_relate_yaw;
        safe_cmd.target_length = chassis_comm.ctrl_cmd.target_length;
        safe_cmd.roll_angle = chassis_comm.ctrl_cmd.roll_angle;
        return &safe_cmd;
    }
}
/* 获取当前控制数据（只读） */
const gimbal_ctrl_frame_t* chassis_get_ctl(void)
{
    return &chassis_comm.ctrl_cmd;
}

/* 获取通信状态 */
uint8_t chassis_get_comm_status(void)
{
    return chassis_comm.comm_ok;
}
