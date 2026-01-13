#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bmi088.h"
#include "bsp_tim.h"
#include "chassis_behaviour.h"
#include "monitor.h"
#include "usart.h"
#include "vofa.h"
// 通信超时阈值（单位：ms）
#define COMM_TIMEOUT_MS         100.0f   // 通信超时阈值
#define FRAME_TIMEOUT_MS        20.0f    // 两帧间隔超时阈值
gimbal_chassis_comm_t gimbal_chassis_comm; // 云台与底盘通信结构体

void init_gimbal_comm_status(void);
static  bool update_gimbal_comm_status(float current_time);
static bool get_safe_gimbal_cmd(gimbal_to_chassis_data_t *cmd_out, float current_time);
void chassis_data_to_gimbal_feedback( const chassis_move_t* chassis);
// void Test_MagYaw(ist8310_real_data_t *ist8310_Info,INS_Info_Typedef *INS_Info);
void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);
    // uint16_t buzzer=1;
    TickType_t systick = 0;
    init_gimbal_comm_status();
    static float last_refresh_dog_time = 0.0f;
    static float last_led_time = 0.0f;
    float current_time = 0.0f;
    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;
    extern Chassis_RC_Info_t chassis_can_rc_info;

    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t* Usb_receive_data;

    extern lk9025_motor_measure_t motor_right, motor_left;
    extern dm8009_motor_measure_t motor_joint[4];
    const SlipDetector_t *local_detector= get_slip_detector_point();
    const chassis_move_t* local_chassis = get_chassis_control_point();
    const Quaternion_Info_Typedef* local_Quaternion_Info = get_quaternion_info_point();
    const LegPredictor_t *leg_predictor = get_leg_predictor_point();

    //监测
    const SystemMonitor_t *monitor = SystemMonitor_Get();

    // extern LegPredictor_t leg_predictor;
    // float q0 = Quaternion_Info.quat[0], q1 = Quaternion_Info.quat[1], q2 = Quaternion_Info.quat[2], q3 = Quaternion_Info.quat[3];
    // float e0 = Quaternion_Info.EulerAngle[0], e1 = Quaternion_Info.EulerAngle[1], e2 = Quaternion_Info.EulerAngle[2];
    // float gx = BMI088_Info.gyro[0], gy = BMI088_Info.gyro[1], gz = BMI088_Info.gyro[2];
		//float leg_cal[2];
    //Usb_send_data_t Usb_send_data_t;
    //Usb_dpkg_data_t* cnm = getUsbDpkgData();
    float cpu_d=0.0f;
    uint32_t led_update_counter=0; //LED更新计数器
    // float wz,wz_fliter;
    // const float num = 0.2f;
    // first_order_filter_type_t filter_t = {.input = wz, .frame_period=0.05f, .out=wz_fliter, .num=num};
    // first_order_filter_init(&filter_t,0.05f,&num);
    for(;;)
    {
        systick = osKernelSysTick();
        current_time=DWT_GetTimeline_ms();

        update_gimbal_comm_status(current_time);
        get_safe_gimbal_cmd(&gimbal_chassis_comm.gimbal_cmd,current_time);
        chassis_data_to_gimbal_feedback(local_chassis);

        //每隔1s就要喂狗，防止1.5s看门狗复位
        if ((current_time-last_refresh_dog_time)>=1300)
        {
            last_refresh_dog_time=current_time;
            SystemMonitor_WatchdogRefresh();

        }
        // LED更新（每50ms调用一次，即每25个循环）
        // led_update_counter++;
        if ((current_time-last_led_time)>=50)
        {
            last_led_time=current_time;
            SystemMonitor_PeriodicTask();

            // led_update_counter = 0;
        }


        // cpu_d=Monitor_GetCPULoad();
            // uart_printf(&huart6, "heart_cont:%d\r\n", monitor_heartbeat->heartbeat_count);
            // uart_printf(&huart6, "heart_status:%d\r\n", monitor_heartbeat->status);
            // uart_printf(&huart6, "heart_restart_reason:%d\r\n", monitor_heartbeat->restart_reason);
            // uart_printf(&huart6, "watchdog_restart_count:%d\r\n", monitor_heartbeat->watchdog_restart_count);
            // uart_printf(&huart6, "cpu:%f\r\n", monitor->cpu.cpu_load_percent);

        // Monitor_Watchdog_Refresh();
        // Test_MagYaw(&ist8310_Info,&INS_Info);
        //printf("%.2f,%.2f\r\n",local_chassis->chassis_roll,local_chassis->chassis_roll_set);
        // Vofa_Send_INS(&huart6,INS_Info,ist8310_Info);
        // Vofa_Send_Q(&huart6,INS_Info,local_Quaternion_Info);
        // Vofa_Send_Chassis(&huart6,INS_Info,motor_joint,local_chassis);


        // Vofa_Send_Chassis_CMD(&huart1, &chassis_can_rc_info,local_chassis);
        // Vofa_Send_Data(&huart1,local_chassis);

        // Vofa_Send_Calibrate(&huart6,local_chassis);
        // Vofa_Send_Slip(&huart6,local_chassis,local_detector);
        // Vofa_Send_Balance(&huart6,local_chassis);
        // Vofa_Send_Pred(&huart1,local_chassis);
        // Vofa_Send_Theata(&huart6,local_chassis);
        // Vofa_Send_Theata_pre(&huart6,local_chassis ,leg_predictor,local_detector);
        if (switch_is_down(remote_ctrl.rc.s[0]))
        {
            buzzer_off();
            // buzzer=1;
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_up(remote_ctrl.rc.s[0]))
        {
            // buzzer_on(84,10);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_mid(remote_ctrl.rc.s[0]))
        {
            // buzzer_on(100);
            // if (buzzer>0)
            // {
            //     buzzer=0;
            //     buzzer_on(84,10);
            // }
            // else
            // {
            //     buzzer_off();
            // }
            // if (systick<10000)buzzer_on(84,10);
            // else buzzer_off();
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
        // printf("%.2f,%.2f\r\n",local_chassis->right_leg.leg_angle,local_chassis->left_leg.leg_angle);
        osDelay(1);
    }
}

// void Test_MagYaw(ist8310_real_data_t *ist8310_Info,INS_Info_Typedef *INS_Info)
// {
//     // 测试数据：IMU水平指向北
//
//     float yaw = ComputeMagYaw(ist8310_Info->calibrated_mag, INS_Info->rol_angle, INS_Info->pit_angle);
//     uart_printf(&huart6, "期望:0°, 实际:%.1f°\r\n", yaw*57.3f);
//
//     // // 测试绕Z轴旋转90度（应该指向东）
//     // float mag_east[3] = {0.0f, -0.2f, 0.4f};  // 假设
//     // float yaw_east = ComputeMagYaw(mag_east, roll, pitch);
//     // uart_printf(&huart6, "水平指向东 - 期望:90°, 实际:%.1f°\n", yaw_east*57.3f);
// }
/**
 * @brief 初始化云台通信状态
 */
void init_gimbal_comm_status(void)
{
    float init_time = DWT_GetTimeline_ms();

    // 初始化时间戳为当前时间，确保初始不触发断联
    gimbal_chassis_comm.last_frame1_time = init_time;
    gimbal_chassis_comm.last_frame2_time = init_time;

    // 初始状态：等待接收
    gimbal_chassis_comm.frame1_received = 0;
    gimbal_chassis_comm.frame2_received = 0;
    gimbal_chassis_comm.comm_ok = 0;
    gimbal_chassis_comm.safe_mode = 1;  // 初始为安全模式

    // 初始化安全命令
    memset(&gimbal_chassis_comm.gimbal_cmd, 0, sizeof(gimbal_to_chassis_data_t));
    gimbal_chassis_comm.gimbal_cmd.chassis_mode_cmd = CHASSIS_ZERO_FORCE;
}
/**
 * @brief 检查并更新云台通信状态
 * @param current_time 当前时间(ms)
 * @return 通信是否正常 (1=正常, 0=断联)
 */
static  bool update_gimbal_comm_status(float current_time)
{
    static uint8_t last_state = 0;

    // 计算时间差
    float dt1 = current_time - gimbal_chassis_comm.last_frame1_time;
    float dt2 = current_time - gimbal_chassis_comm.last_frame2_time;

    // 通信正常条件：
    // 1. 两帧都在20ms内收到
    // 2. 两帧都已收到标志位为1
    uint8_t new_state = (dt1 < 20.0f) &&
                        (dt2 < 20.0f) &&
                        gimbal_chassis_comm.frame1_received &&
                        gimbal_chassis_comm.frame2_received;

    // 状态变化处理
    if (new_state != last_state) {
        gimbal_chassis_comm.comm_ok = new_state;
        gimbal_chassis_comm.safe_mode = !new_state;
        last_state = new_state;

        // // 可选：调试输出
        // if (new_state) {
        //     printf("Gimbal comm OK\n");
        // } else {
        //     printf("Gimbal comm LOST\n");
        // }
    }

    return new_state;
}
/**
 * @brief 获取安全的控制命令（自动处理断联）
 * @param cmd_out 输出命令指针
 * @param current_time 当前时间(ms)
 * @return 是否是正常通信的命令 (1=正常, 0=安全模式)
 */
static bool get_safe_gimbal_cmd(gimbal_to_chassis_data_t *cmd_out, float current_time)
{
    uint8_t comm_ok = update_gimbal_comm_status(current_time);

    if (comm_ok) {
        // 通信正常：直接复制命令
        // *cmd_out = gimbal_chassis_comm.gimbal_cmd;
        return 1;
    } else {
        // 通信断联：安全命令
        cmd_out->chassis_mode_cmd = CHASSIS_ZERO_FORCE;
        cmd_out->target_speed_x = 0.0f;
        cmd_out->target_speed_w_z = 0.0f;
        cmd_out->target_length = 0.0f;
        cmd_out->roll_angle = 0.0f;
        cmd_out->spinning_cmd = 0;
        cmd_out->jump_cmd = 0;
        return 0;
    }
}
void chassis_data_to_gimbal_feedback( const chassis_move_t* chassis)
{
    gimbal_chassis_comm.chassis_feedback.chassis_online=1;
    gimbal_chassis_comm.chassis_feedback.chassis_yaw_angle=chassis->chassis_yaw;
    gimbal_chassis_comm.chassis_feedback.chassis_yaw_rate=chassis->chassis_imu_gyro[0];//逆时针为正
    gimbal_chassis_comm.chassis_feedback.current_speed_x=chassis->state_ref.x_dot;
    gimbal_chassis_comm.chassis_feedback.current_speed_w_z=chassis->wz;
    gimbal_chassis_comm.chassis_feedback.jump_state=chassis->jump_state.jump_flag;
    gimbal_chassis_comm.chassis_feedback.chassis_mode_current=chassis->chassis_mode;
    gimbal_chassis_comm.chassis_feedback.spinning_state=chassis->spining_flag;
}
/**
 * @brief 解析云台控制命令第一帧
 * @param cmd 指向控制命令结构的指针
 * @param data CAN接收到的8字节数据数组
 */
void chassis_parse_control_frame1(gimbal_to_chassis_data_t *cmd, uint8_t data[8])
{
    if (!cmd || !data) return;

    // 检查帧标志位
    if (data[7] != 0x00) return;  // 不是第一帧

    // 解析控制命令
    cmd->chassis_mode_cmd = data[0] & 0x0F;
    cmd->spinning_cmd = (data[0] >> 4) & 0x01;
    cmd->jump_cmd = (data[0] >> 5) & 0x01;

    // 解析云台YAW角度（精度0.001 rad）
    int16_t yaw_int = (int16_t)((data[2] << 8) | data[1]);
    cmd->gimbal_yaw_angle = yaw_int / 1000.0f;

    // 解析前进速度（精度0.001 m/s）
    int16_t speed_x_int = (int16_t)((data[4] << 8) | data[3]);
    cmd->target_speed_x = speed_x_int / 1000.0f;

    // 解析旋转速度（精度0.001 rad/s）
    int16_t wz_int = (int16_t)((data[6] << 8) | data[5]);
    cmd->target_speed_w_z = wz_int / 1000.0f;

}

/**
 * @brief 解析云台控制命令第二帧
 * @param cmd 指向控制命令结构的指针
 * @param data CAN接收到的8字节数据数组
 */
void chassis_parse_control_frame2(gimbal_to_chassis_data_t *cmd, uint8_t data[8])
{
    if (!cmd || !data) return;

    // 检查帧标志位
    if (data[7] != 0x01) return;  // 不是第二帧

    // 解析腿长（精度0.001 m）
    int16_t length_int = (int16_t)((data[1] << 8) | data[0]);
    cmd->target_length = length_int / 1000.0f;

    // 解析Roll角度（精度0.001 rad）
    int16_t roll_int = (int16_t)((data[3] << 8) | data[2]);
    cmd->roll_angle = roll_int / 1000.0f;

}


const gimbal_chassis_comm_t* get_gimbal_chassis_comm_point(void)
{
    return &gimbal_chassis_comm;
}