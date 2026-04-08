/**
 * @file Gimbal_task.c
 * @brief 云台控制任务源文件
 * @details 实现云台PID控制、模式切换、发射系统控制等功能
 * @author RM Team
 * @date 2025
 */

#include "Gimbal_task.h"
#include "ctl_chassis.h" /* gimbal_ctl_chassis_cmd 已移至底盘通信设备模块 */
#include "buzzer_music.h"
#include "main.h"
#include "monitor.h"
#include "old_pid.h"
#include "User_Task.h"

/**
 * @brief 遥控器死区限制宏
 * @details 对遥控器输入值进行死区处理，小于死区值时输出为0
 * @param input 输入值
 * @param output 输出值
 * @param dealine 死区阈值
 */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/* 全局变量定义 */
static gimbal_t gimbal;                        // 云台控制结构体实例（static: 只允许本 TU 内写入，外部通过 get_gimbal_point() 只读）
static volatile bool gimbal_init_done = false; // 初始化完成标志，置位后其他任务才可安全访问 gimbal 内指针
#define YAW_ZERO_ECD 8105U
// extern uint8_t Usart_Mode;                     // 串口控制模式

/* 函数声明 */
void init_gimbal_control(gimbal_t *gimbal);                      // 云台控制初始化
void gimbal_set_mod(gimbal_t *gimbal_set_mod);                   // 设置云台工作模式
void gimbal_mod_change_date_transfer(gimbal_t *gimbal_transfer); // 模式切换数据转换
void gimbal_feedback(gimbal_t *gimbal_feedback);                 // 云台反馈数据更新
void gimbal_set_control(gimbal_t *gimbal_set_control);           // 设置云台控制量
void gimbal_control_loop(gimbal_t *gimbal_loop);                 // 云台控制循环
static void limit_pitch_command_by_encoder(gimbal_t *gimbal, float *pitch_delta);
void update_gimbal_spinning_mode(gimbal_t *gimbal);              // 更新云台小陀螺模式状态

/* 新增控制函数声明 */
static void gimbal_no_force_control(gimbal_t *gimbal);           // 无力模式控制
static void gimbal_pitch_control(gimbal_t *gimbal);              // PITCH轴控制
static void gimbal_slow_cali_control(gimbal_t *gimbal);          // 慢速校准模式控制
static void gimbal_spinning_mode_control(gimbal_t *gimbal, float chassis_yaw_rate); // 小陀螺模式控制
static void gimbal_follow_chassis_control(gimbal_t *gimbal, float chassis_yaw_rate); // 云台跟随底盘模式控制
static void gimbal_normal_mode_control(gimbal_t *gimbal);        // 正常模式控制
/* gimbal_ctl_chassis_cmd 已移至 ctl_chassis.c 统一管理，通过 ctl_chassis.h 引入 */
/**
 * @brief 云台控制任务主函数
 * @param argument FreeRTOS任务参数
 * @details 云台控制任务的主循环，负责模式设置、数据更新、控制计算等
 */
void Gimbal_Task(void const *argument)
{
    /* 等待底层硬件外设（SPI/CAN/UART HAL）完成初始化，通常几十毫秒已足够 */
    osDelay(500);
    TickType_t systick = 0;

    // 初始化云台控制系统
    init_gimbal_control(&gimbal);
    gimbal_init_done = true; // 置位：通知其他任务 gimbal 内所有指针已有效
    // boot_play_music();
    /* 无限循环 */
    for (;;)
    {
        systick = osKernelSysTick();

        gimbal_set_mod(&gimbal);                  // 设置云台工作模式
        gimbal_mod_change_date_transfer(&gimbal); // 模式切换时的数据转换
        gimbal_feedback(&gimbal);                 // 更新反馈数据
        gimbal_set_control(&gimbal);              // 设置控制目标
        gimbal_control_loop(&gimbal);             // 执行控制循环

        gimbal_ctl_chassis_cmd(&gimbal); // 发送云台控制命令到底盘
        SystemMonitor_Process();   // 系统监控处理  - 心跳和CPU记录（每2ms调用）
        osDelayUntil(&systick, 2); // 2ms周期控制
    }
}

/**
 * @brief 初始化云台发射系统
 * @param shoot 发射系统结构体指针
 * @details 初始化发射轮和拨弹轮的PID参数，获取电机指针，设置初始状态
 */
void init_gimbal_shoot(gimbal_shoot_t *shoot)
{

    // 拨弹轮PID参数数组
    const static fp32 pull_w_pid[3] = {PID_SHOOT_PULL_MOTOR_KP, PID_SHOOT_PULL_MOTOR_KI, PID_SHOOT_PULL_MOTOR_KD};
    // 右发射轮PID参数数组
    const static fp32 shoot_w_r_pid[3] = {PID_SHOOT_RIGHT_MOTOR_KP, PID_SHOOT_RIGHT_MOTOR_KI, PID_SHOOT_RIGHT_MOTOR_KD};
    // 左发射轮PID参数数组
    const static fp32 shoot_w_l_pid[3] = {PID_SHOOT_LEFT_MOTOR_KP, PID_SHOOT_LEFT_MOTOR_KI, PID_SHOOT_LEFT_MOTOR_KD};

    // 初始化各电机PID控制器
    old_PID_Init(&shoot->Shoot_right_PID, PID_POSITION, shoot_w_r_pid, PID_SHOOT_RIGHT_MOTOR_MAXOUT, PID_SHOOT_RIGHT_MOTOR_MAXI);
    old_PID_Init(&shoot->Shoot_left_PID, PID_POSITION, shoot_w_l_pid, PID_SHOOT_LEFT_MOTOR_MAXOUT, PID_SHOOT_LEFT_MOTOR_MAXI);
    old_PID_Init(&shoot->Pull_PID, PID_POSITION, pull_w_pid, PID_SHOOT_PULL_MOTOR_MAXOUT, PID_SHOOT_PULL_MOTOR_MAXI);

    // 获取电机测量数据指针
    shoot->pull_motor = get_shoot_motor_pull();         // 拨弹轮电机
    shoot->shoot_motor_right = get_shoot_motor_right(); // 右发射轮电机
    shoot->shoot_motor_left = get_shoot_motor_left();   // 左发射轮电机
    shoot->shoot_target_speed = 0.0f;                   // 初始目标转速为0

    shoot->shoot_motor_left->target_current = 0.0f;
    shoot->shoot_motor_right->target_current = 0.0f;
    shoot->pull_motor->target_current = 0.0f;
    // 初始化射击状态机
    shoot->shoot_mode = SHOOT_STOP;
    shoot->bullet_done = false;
    shoot->block_count = 0;
    shoot->reverse_count = 0;
}

/**
 * @brief 初始化云台位置控制系统
 * @param pos_control 位置控制结构体指针
 * @details 初始化滤波器、PID控制器、获取电机指针、设置初始状态
 */
void init_gimbal_pos_control(gimbal_pos_control_t *pos_control)
{

    // 滤波器参数设置
    const static fp32 gimbal_yaw_order_filter[1] = {0.166f};   // YAW轴遥控滤波器参数
    const static fp32 gimbal_pitch_order_filter[1] = {0.166f}; // PITCH轴遥控滤波器参数

    const static fp32 gimbal_yaw_pos_filter[1] = {0.2f};   // YAW轴位置滤波器参数
    const static fp32 gimbal_pitch_pos_filter[1] = {0.2f}; // PITCH轴位置滤波器参数

    // 初始化一阶低通滤波器
    first_order_filter_init(&pos_control->filter_rc_yaw_vel_set, 0.002f, gimbal_yaw_order_filter);     // 遥控YAW速度滤波器
    first_order_filter_init(&pos_control->filter_rc_pitch_vel_set, 0.002f, gimbal_pitch_order_filter); // 遥控PITCH速度滤波器

    first_order_filter_init(&pos_control->filter_MiniPc_pitch_set, 0.002, gimbal_pitch_pos_filter); // 小电脑PITCH位置滤波器
    first_order_filter_init(&pos_control->filter_MiniPc_yaw_set, 0.002, gimbal_yaw_pos_filter);     // 小电脑YAW位置滤波器

    // 初始化PITCH轴PID控制器
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};                            // 速度环PID参数
    static const fp32 Pitch_absolute_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->gimbal_motor_absolute_angle_pid, PID_POSITION, Pitch_absolute_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    // 初始化YAW轴PID控制器
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};                            // 速度环PID参数
    static const fp32 Yaw_absolute_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD}; // 绝对角度环PID参数
    static const fp32 Yaw_relative_pid[3] = {YAW_GYRO_RELATIVE_PID_KP, YAW_GYRO_RELATIVE_PID_KI, YAW_GYRO_RELATIVE_PID_KD}; // 相对角度环PID参数
    old_PID_Init(&pos_control->yaw_absolute_angle_pid, PID_POSITION, Yaw_absolute_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->yaw_relative_angle_pid, PID_POSITION, Yaw_relative_pid, YAW_GYRO_RELATIVE_PID_MAX_OUT, YAW_GYRO_RELATIVE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->yaw_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    // 初始化目标位置
    pos_control->yaw_target_pos = 0.0f;
    pos_control->pitch_target_pos = 0.0f;

    // 获取电机测量数据指针
    pos_control->pitch_motor_measure = get_pitch_motor();
    pos_control->yaw_motor_measure = get_yaw_motor();

    // 初始化状态标志
    pos_control->calibrate_warning = false;

    // 设置初始电流为0，初始化相对位置
    pos_control->pitch_motor_measure->target_tor = 0;
    pos_control->yaw_motor_measure->target_current = 0;
    pos_control->yaw_relattive_pos = 0.0f;

    // 获取USB自瞄数据指针
    pos_control->usb_autoAim_ptr = getUsbMiniPcPtr();
}
/**
 * @brief 初始化云台控制系统
 * @param gimbal 云台控制结构体指针
 * @details 调用各子系统初始化函数，获取外部数据指针，设置初始状态
 */
void init_gimbal_control(gimbal_t *gimbal)
{
    // 初始化发射系统
    init_gimbal_shoot(&gimbal->gimbal_shoot);
    // 初始化位置控制系统
    init_gimbal_pos_control(&gimbal->gimbal_pos);

    // 获取外部数据指针
#ifdef USE_SBUS_PROTOCOL
    gimbal->gimbal_RC = get_sbus_remote_control_point(); // 富斯i6x遥控器数据
#else
    // gimbal->gimbal_RC = get_remote_control_point(); // DT7遥控器数据
#endif
    gimbal->gimbal_PC_RC = get_pc_uart_ctrl_point(); // 图传链路数据
    gimbal->gimbal_mod = GIMBAL_MOD_NO_FORCE;        // 初始模式为无力
    gimbal->minipc_info = getUsbDpkgData();          // 小电脑通信数据
    gimbal->ins_info = get_ins_info_point();         // 惯性导航数据

    // 更新反馈数据并设置初始位置
    gimbal_feedback(gimbal);
    gimbal->gimbal_pos.pitch_target_pos = gimbal->gimbal_pos.pitch_absolute_pos; // PITCH目标位置为当前角度
    gimbal->gimbal_pos.init_pitch_pos = gimbal->gimbal_pos.pitch_relatiive_pos;  // 记录PITCH编码器零位参考
    gimbal->gimbal_pos.init_yaw_pos = gimbal->gimbal_pos.yaw_relattive_pos;
    gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.yaw_relattive_pos;

    // 初始化其他状态变量
    gimbal->gimbal_shoot.pull_target_speed = SHOOT_PULL_FORWARD_SPEED; // 拨弹轮初始目标速度
    gimbal->gimbal_pos.add_pitch = 0.0f;                               // PITCH增量清零
    gimbal->gimbal_pos.add_yaw = 0.0f;                                 // YAW增量清零
}
///////////////////////////初始化结束//////////////////
/////////////////////////模式设置开始./////////////
// #ifdef USE_SBUS_PROTOCOL
// #else
// #endif
/**
 * @brief 设置云台工作模式
 * @param gimbal_set_mod 云台控制结构体指针
 * @details 根据遥控器拨杆位置或串口命令设置云台工作模式，支持模式切换逻辑
 */
void gimbal_set_mod(gimbal_t *gimbal_set_mod)
{
    static TickType_t cal_time = 0; // 校准时间记录

#ifdef USE_RC_CONTROL

#ifdef USE_SBUS_PROTOCOL

    // 左1拨杆上拨（=-1）：无力模式
    if (sbus_sw_is_up(gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]))
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    // 左1拨杆下拨（=+1）且当前为无力模式：进入慢速校准模式
    else if (sbus_sw_is_down(gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time = xTaskGetTickCount();
    }
    // 左1拨杆下拨（=+1）且当前为慢速校准模式：校准完成后进入正常模式
    else if (sbus_sw_is_down(gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
    {
        if (xTaskGetTickCount() - cal_time < 1200)
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        }
        else
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 左2拨杆上拨（=-1）：退出自瞄
    else if (sbus_sw_is_up(gimbal_set_mod->gimbal_RC->rc.s[LEFT_2_SWITCH]))
    {
        // gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
        gimbal_set_mod->anto_aim_mode_cmd = false;
    }
    // 左2拨杆下拨（=+1）：自动瞄准模式
    else if (sbus_sw_is_down(gimbal_set_mod->gimbal_RC->rc.s[LEFT_2_SWITCH]) && gimbal_set_mod->gimbal_mod != GIMBAL_MOD_NO_FORCE && gimbal_set_mod->last_anto_aim_mode_cmd == false)
    {
        gimbal_set_mod->anto_aim_mode_cmd = true;
    }
    // 右2拨杆中位（MID=0）且正常模式：进入云台跟随底盘模式
    else if (sbus_sw_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_2_SWITCH]) && gimbal_set_mod->last_gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_FLOW_CHASSIS;
    }
    // 右2拨杆下拨（=+1）且跟随模式：退出云台跟随底盘，进入正常模式
    else if (sbus_sw_is_down(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_2_SWITCH]) && gimbal_set_mod->last_gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    //////////////////////////////////////////////////////////////////////
    if (gimbal_set_mod->gimbal_RC->rc.ch[RC_ROLL_1_CH] < -200 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_set_mod->gimbal_shoot.fire_flag = true;
    }
    else
    {
        gimbal_set_mod->gimbal_shoot.fire_flag = false;
    }
    // 遥控器上次控制模式更新
    gimbal_set_mod->last_gimbal_mod = gimbal_set_mod->gimbal_mod;
    gimbal_set_mod->last_anto_aim_mode_cmd = gimbal_set_mod->anto_aim_mode_cmd;
    //////////////////////////////////////////////////////////////////////
    // 底盘控制命令设置

    //////////////////////////////////////////////////////////////////////
#else

    // 右拨杆下拨：无力模式
    if (switch_is_down(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]))
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    // 右拨杆中位且当前为无力模式：进入慢速校准模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time = xTaskGetTickCount();
    }
    // 右拨杆中位且当前为慢速校准模式：校准1.5秒后进入正常模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
    {
        if (xTaskGetTickCount() - cal_time < 1500)
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        }
        else
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 左拨杆中位且当前为自瞄模式：退出自瞄，进入正常模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[LEFT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_AutoAim)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    // 左拨杆上拨：自动瞄准模式
    else if (switch_is_up(gimbal_set_mod->gimbal_RC->rc.s[LEFT_SWITCH]))
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoAim;
    }
    // 右拨杆上拨且当前为正常模式：进入云台跟随底盘模式
    else if (switch_is_up(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_FLOW_CHASSIS;
    }
    // 右拨杆中位且当前为云台跟随底盘模式：退出云台跟随底盘，进入正常模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
#endif
#endif
    // 模式键0挡位：无力模式
    if (gimbal_set_mod->gimbal_PC_RC->rc.mode_sw == 0)
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    // 模式键1挡位且当前为无力模式：进入慢速校准模式
    else if ((gimbal_set_mod->gimbal_PC_RC->rc.mode_sw == 1) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time = xTaskGetTickCount();
    }
    // 模式键1挡位且当前为慢速校准模式：校准0.5秒后进入正常模式
    else if ((gimbal_set_mod->gimbal_PC_RC->rc.mode_sw == 1) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
    {
        if (xTaskGetTickCount() - cal_time < 500)
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        }
        else
        {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 左fn_1按下为1且当前为自瞄模式：退出自瞄，进入正常模式
    else if (gimbal_set_mod->gimbal_PC_RC->rc.fn_1 == 1 && (gimbal_set_mod->PC_fn1_last == false) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_AutoAim)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    // 左fn_1按下为1：切换并进入自动瞄准模式
    else if (gimbal_set_mod->gimbal_PC_RC->rc.fn_1 == 1 && (gimbal_set_mod->PC_fn1_last == false) && gimbal_set_mod->gimbal_mod != GIMBAL_MOD_AutoAim)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoAim;
    }
    // 模式键2挡位且当前为正常模式：进入云台跟随底盘模式
    else if (gimbal_set_mod->gimbal_PC_RC->rc.mode_sw == 2 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_FLOW_CHASSIS;
    }
    // 模式键1且当前为云台跟随底盘模式：退出云台跟随底盘，进入正常模式
    else if (gimbal_set_mod->gimbal_PC_RC->rc.mode_sw == 1 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS)
    {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    ///// 小陀螺切换：右fn_2按住 且处于正常模式时激活 //////////
    if (gimbal_set_mod->gimbal_PC_RC->rc.fn_2 == 1 && (gimbal_set_mod->PC_fn2_last == false) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL&& gimbal_set_mod->spinning_mode_cmd == false)
    {
        // gimbal_set_mod->chassis_cmd.spinning_cmd = true;
        gimbal_set_mod->spinning_mode_cmd = true;
    }
    else if (gimbal_set_mod->gimbal_PC_RC->rc.fn_2 == 1 && (gimbal_set_mod->PC_fn2_last == false) && gimbal_set_mod->spinning_mode_cmd == true)
    {
        // gimbal_set_mod->chassis_cmd.spinning_cmd = false;
        gimbal_set_mod->spinning_mode_cmd = false;
    }
    /////跳跃指令：pause按住 且处于正常模式时激活 //////////
    if (gimbal_set_mod->gimbal_PC_RC->rc.pause == 1 && (gimbal_set_mod->PC_pause_last == false) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_set_mod->chassis_cmd.jump_cmd = true;
    }
    else if (gimbal_set_mod->gimbal_PC_RC->rc.pause == 1 && (gimbal_set_mod->PC_pause_last == false) && gimbal_set_mod->chassis_cmd.jump_cmd == true)
    {
        gimbal_set_mod->chassis_cmd.jump_cmd = false;
    }
    if (gimbal_set_mod->gimbal_pos.usb_autoAim_ptr->control_flag.bits.exit_auto_aim ||
        !gimbal_set_mod->gimbal_pos.usb_autoAim_ptr->control_flag.bits.auto_aim)
    {
        gimbal_set_mod->anto_aim_mode_cmd = false;
    }
    else if (gimbal_set_mod->gimbal_pos.usb_autoAim_ptr->control_flag.bits.auto_aim &&
             gimbal_set_mod->gimbal_mod != GIMBAL_MOD_NO_FORCE)
    {
        gimbal_set_mod->anto_aim_mode_cmd = true;
    }
    // 更新上次遥控器模式状态
    gimbal_set_mod->PC_fn1_last = gimbal_set_mod->gimbal_PC_RC->rc.fn_1;
    gimbal_set_mod->PC_fn2_last = gimbal_set_mod->gimbal_PC_RC->rc.fn_2;
    gimbal_set_mod->PC_pause_last = gimbal_set_mod->gimbal_PC_RC->rc.pause;
    gimbal_set_mod->last_spinning_mode_cmd = gimbal_set_mod->spinning_mode_cmd;
    // 更新云台小陀螺模式状态
    // update_gimbal_spinning_mode(gimbal_set_mod);
}

/**
 * @brief 云台模式切换时的数据转换处理
 * @param gimbal_transfer 云台控制结构体指针
 * @details 在不同工作模式之间切换时，进行必要的数据转换和目标位置设置
 */
void gimbal_mod_change_date_transfer(gimbal_t *gimbal_transfer)
{
    static gimbal_mod_e last_pid_reset_mod = GIMBAL_MOD_NO_FORCE;

    if (last_pid_reset_mod != gimbal_transfer->gimbal_mod)
    {
        if (gimbal_transfer->gimbal_mod == GIMBAL_MOD_NORMAL ||
            gimbal_transfer->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS ||
            gimbal_transfer->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
        {
            old_PID_clear(&gimbal_transfer->gimbal_pos.yaw_absolute_angle_pid);
            old_PID_clear(&gimbal_transfer->gimbal_pos.yaw_relative_angle_pid);
            old_PID_clear(&gimbal_transfer->gimbal_pos.yaw_motor_gyro_pid);
        }
        last_pid_reset_mod = gimbal_transfer->gimbal_mod;
    }

    // 无力模式：目标位置跟随当前位置
    if (gimbal_transfer->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->ins_info->pit_angle;
        gimbal_transfer->gimbal_pos.init_pitch_pos = gimbal_transfer->gimbal_pos.pitch_relatiive_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
    }
    // 从无力模式切换到慢速校准模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_NO_FORCE && gimbal_transfer->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
    {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->ins_info->pit_angle;
        gimbal_transfer->gimbal_pos.init_pitch_pos = gimbal_transfer->gimbal_pos.pitch_relatiive_pos;

        // 初始化YAW目标为当前相对位置，防止大幅度跳变震荡
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
    }
    // 从慢速校准模式切换到正常控制模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_SLOW_CALI && gimbal_transfer->gimbal_mod == GIMBAL_MOD_NORMAL)
    {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
    }
    // 切换到自动瞄准模式
    else if (gimbal_transfer->last_anto_aim_mode_cmd == false && gimbal_transfer->anto_aim_mode_cmd == true)
    {
        // 保留视觉目标设置的代码（已注释）
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
    }
    // 切换到云台跟随底盘模式
    else if (gimbal_transfer->last_gimbal_mod != GIMBAL_MOD_FLOW_CHASSIS && gimbal_transfer->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS)
    {
        // 云台跟随底盘模式：云台yaw保持相对底盘位置为0，即与底盘同向
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
    }

    // 从云台跟随底盘模式切换到其他模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS && gimbal_transfer->gimbal_mod != GIMBAL_MOD_FLOW_CHASSIS)
    {
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
    }
    // 激活小陀螺模式，保持当前相对位置为目标位置，防止模式切换时云台大幅度跳变
    if (gimbal_transfer->spinning_mode_cmd==true&& gimbal_transfer->last_spinning_mode_cmd==false)
    {
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_relattive_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
    }
    // 退出小陀螺模式，保持当前绝对位置为目标，防止模式切换时云台大幅度跳变
    else if (gimbal_transfer->spinning_mode_cmd==false&& gimbal_transfer->last_spinning_mode_cmd==true)
    {
        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
    }
    // // 小陀螺模式切换：激活时保持当前绝对位置为目标，退出时保持当前绝对位置为目标
    // if (gimbal_transfer->spinning_mode_cmd==false&& gimbal_transfer->last_spinning_mode_cmd==true)
    // {
    //     gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
    // }
}
// 计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

/**
 * @brief 更新云台反馈数据
 * @param gimbal_feedback 云台控制结构体指针
 * @details 从IMU和电机编码器获取当前位置信息，更新云台和发射系统的反馈数据
 */
void gimbal_feedback(gimbal_t *gimbal_feedback)
{

    // 更新PITCH轴绝对位置（来自IMU）
    gimbal_feedback->gimbal_pos.pitch_absolute_pos = gimbal_feedback->ins_info->pit_angle;
    gimbal_feedback->gimbal_pos.pitch_relatiive_pos = gimbal_feedback->gimbal_pos.pitch_motor_measure->pos;
    // gimbal_feedback->gimbal_pos.pitch_absolute_pos = rad_format(gimbal_feedback->gimbal_pos.pitch_absolute_pos);

    // 更新YAW轴绝对位置（来自IMU）
    gimbal_feedback->gimbal_pos.yaw_absolute_pos = gimbal_feedback->ins_info->yaw_angle;
    gimbal_feedback->gimbal_pos.yaw_absolute_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_absolute_pos);

    // 更新YAW轴相对位置（来自电机编码器）
    // 当电机-1.778651f角度时，编码器值为2319，此时枪管正对前方，作为相对零位
    gimbal_feedback->gimbal_pos.yaw_relattive_pos = motor_ecd_to_angle_change(gimbal_feedback->gimbal_pos.yaw_motor_measure->pos, YAW_ZERO_ECD);
    gimbal_feedback->gimbal_pos.yaw_relattive_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_relattive_pos);
    gimbal_feedback->gimbal_pos.yaw_motor_measure->real_pos = gimbal_feedback->gimbal_pos.yaw_relattive_pos;
    gimbal_feedback->gimbal_pos.yaw_motor_measure->real_w = gimbal_feedback->gimbal_pos.yaw_motor_measure->rpm * 2.0f * PI / 60.0f;
    // gimbal_feedback->gimbal_pos.yaw_relattive_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_relattive_pos);

    // 更新发射系统反馈数据（将RPM转换为rad/s）
    gimbal_feedback->gimbal_shoot.shoot_pull_rad = gimbal_feedback->gimbal_shoot.pull_motor->rpm * 2.0f * PI / (36.0f * 60.0f);       // 拨弹轮（36:1减速比）
    gimbal_feedback->gimbal_shoot.shoot_left_rad = -1.0f * gimbal_feedback->gimbal_shoot.shoot_motor_left->rpm * 2.0f * PI / (60.0f); // 左发射轮
    gimbal_feedback->gimbal_shoot.shoot_right_rad = gimbal_feedback->gimbal_shoot.shoot_motor_right->rpm * 2.0f * PI / (60.0f);       // 右发射轮
}
///////////////////////set mod end///////////////

/**
 * @brief 遥控器控制云台设置
 * @param gimbal 云台控制结构体指针
 * @param add_yaw YAW轴增量输出指针
 * @param add_pitch PITCH轴增量输出指针
 * @details 处理遥控器输入，经过死区限制和滤波后输出云台控制增量
 */
void gimbal_rc_set_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch)
{
    if (gimbal == NULL)
    {
        return;
    }

    // 遥控器原始通道值
    int16_t wz_channel, pitch_channel;
    fp32 wz_set_channel, pitch_set_channel;
    bool mouse_control = false; // 是否使用鼠标控制云台
    // #ifdef USE_SBUS_PROTOCOL
    // #else
    // #endif
    // 死区限制，避免遥控器摇杆中位时的微小偏差
#ifdef USE_PC_CONTROL
    if (gimbal->gimbal_PC_RC->rc.ch[RC_LEFT_X_CH]>10||gimbal->gimbal_PC_RC->rc.ch[RC_LEFT_X_CH]<-10||gimbal->gimbal_PC_RC->rc.ch[RC_RIGHT_Y_CH]>10||gimbal->gimbal_PC_RC->rc.ch[RC_RIGHT_Y_CH]<-10)
    {
        rc_deadline_limit(gimbal->gimbal_PC_RC->rc.ch[RC_LEFT_X_CH], wz_channel, 20);    // 左摇杆X轴控制YAW
        rc_deadline_limit(gimbal->gimbal_PC_RC->rc.ch[RC_RIGHT_Y_CH], pitch_channel, 20); // 右摇杆Y轴控制PITCH
        mouse_control = false;
    }
    else
    {
        rc_deadline_limit(gimbal->gimbal_PC_RC->mouse.x, wz_channel, 20);    // 鼠标X轴控制YAW
        rc_deadline_limit(gimbal->gimbal_PC_RC->mouse.y, pitch_channel, 20); // 鼠标Y轴控制PITCH
        mouse_control=true;
    }

#else
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_X_CH], wz_channel, 20);    // 右摇杆X轴控制YAW
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_Y_CH], pitch_channel, 20); // 右摇杆Y轴控制PITCH
#endif
    // 通道值转换为控制量（乘以灵敏度系数）
    if (mouse_control==false)
    {
        wz_set_channel = -wz_channel * GIMBAL_WZ_RC_SEN;
        pitch_set_channel = pitch_channel * GIMBAL_PITCH_RC_SEN;
    }
    else
    {
        wz_set_channel = -wz_channel * GIMBAL_WZ_MOUSE_SEN;
        pitch_set_channel = pitch_channel * GIMBAL_PITCH_MOUSE_SEN;
    }


    // 一阶低通滤波，平滑控制输入
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_yaw_vel_set, wz_set_channel);
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_pitch_vel_set, pitch_set_channel);

    // 输出滤波后的控制增量
    *add_pitch = gimbal->gimbal_pos.filter_rc_pitch_vel_set.out;
    *add_yaw = gimbal->gimbal_pos.filter_rc_yaw_vel_set.out;
}

////////set control start//////////////
/**
 * @brief 云台校准控制
 * @param gimbal 云台控制结构体指针
 * @param add_yaw YAW轴增量输出指针
 * @param add_pitch PITCH轴增量输出指针
 * @details 在校准模式下，将云台缓慢移动到零位
 */

void calibrate_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch)
{

    // 缓慢将PITCH轴移动到零位（除以500减慢速度，指数衰减）
    // *add_pitch = 0.0f -gimbal->gimbal_pos.pitch_target_pos / 800.0f;
    // *add_yaw = 0.0f - gimbal->gimbal_pos.yaw_target_pos / 300.0f;
    //
    // gimbal->gimbal_pos.pitch_target_pos = (0.0f - gimbal->ins_info->pit_angle) / 2;
    // // gimbal->gimbal_pos.yaw_target_pos = (0.0f - gimbal->gimbal_pos.yaw_relattive_pos)/3;
    // // 当PITCH角度接近零位时，直接设置目标为零
    // if (fabs(gimbal->ins_info->pit_angle) < 0.1f)
    // {
    //     // *add_pitch = 0.0f - gimbal->gimbal_pos.pitch_target_pos;
    //     gimbal->gimbal_pos.pitch_target_pos = 0.0f;
    // }
    // if (fabs(gimbal->gimbal_pos.yaw_relattive_pos) < 0.3f)
    // {
    //     *add_yaw = 0.0f - gimbal->gimbal_pos.yaw_target_pos;
    //     // gimbal->gimbal_pos.yaw_target_pos=0.0f;
    // }
    gimbal->gimbal_pos.yaw_target_pos = 0.0f- gimbal->gimbal_pos.yaw_relattive_pos;
}

/**
 * @brief 小电脑（视觉）控制云台
 * @param gimbal 云台控制结构体指针
 * @param add_yaw YAW轴增量输出指针
 * @param add_pitch PITCH轴增量输出指针
 * @details 根据视觉系统提供的目标位置控制云台，处理不同的数据有效性情况
 */
void minipc_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch)
{

    // 只有YAW轴目标有效的情况
    if (isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw))
    {
        *add_pitch = 0.0f;
        *add_yaw = 0.0f;
        // 直接设置YAW目标位置
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw;
    }
    // 只有PITCH轴目标有效的情况
    else if (isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch))
    {
        *add_yaw = 0;
        *add_pitch = 0.0f;
    }
    // YAW和PITCH目标都有效的情况
    else if (!isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch))
    {
        // 直接设置YAW目标位置
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw;
        *add_yaw = 0.0f;

        // 计算PITCH轴增量，缓慢调整（除以200减慢速度）
        gimbal->gimbal_pos.add_pitch = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch - gimbal->gimbal_pos.pitch_target_pos;
        *add_pitch = gimbal->gimbal_pos.add_pitch / 200.0f;
    }
    // 都无效的情况
    else
    {
        *add_yaw = 0.0f;
        *add_pitch = 0.0f;
    }
}

/**
 * @brief 位置限制函数
 * @param pos 位置指针
 * @param MAX 最大值
 * @param MIN 最小值
 * @details 将位置限制在最大值和最小值之间
 */
void limitPos(float *pos, float MAX, float MIN)
{
    if (*pos > MAX)
        *pos = MAX;
    if (*pos < MIN)
        *pos = MIN;
}

static void limit_pitch_command_by_encoder(gimbal_t *gimbal, float *pitch_delta)
{
    const float pitch_encoder_pos = gimbal->gimbal_pos.pitch_relatiive_pos;

    if ((pitch_encoder_pos >= MAX_PITCH_RAD && *pitch_delta > 0.0f) ||
        (pitch_encoder_pos <= MIN_PITCH_RAD && *pitch_delta < 0.0f))
    {
        *pitch_delta = 0.0f;
    }
}

/**
 * @brief 跨零点旋转处理
 * @param target_pos 目标位置指针
 * @param this_time_pos 当前位置指针
 * @details 处理角度跨越±π时的连续性问题，选择最短路径
 */
void cross_roattion_handle(float *target_pos, const float *this_time_pos)
{
    // 如果目标位置与当前位置差值过大，调整目标位置以选择最短路径
    if (*target_pos - *this_time_pos > 2.0f * PI / 3.0f)
    {
        *target_pos -= 2.0f * PI; // 目标位置减去2π
    }
    else if (*target_pos - *this_time_pos < -2.0f * PI / 3.0f)
    {
        *target_pos += 2.0f * PI; // 目标位置加上2π
    }
    else
    {
        *target_pos = rad_format(*target_pos); // 标准化角度到±π范围
    }
}
//////////////////////////////////////////////////////////////////
/*摩擦轮和发射开关*/
bool_t switch_is_fric_on(int16_t ch)
{
    static bool_t fric_flag = 0;

    if (ch < -CH4_HIGH_VALUE)
    {
        fric_flag = 1;
    }
    else if (ch > -CH4_LOW_VALUE)
    {
        fric_flag = 0;
    }

    return fric_flag;
}

bool_t switch_is_shoot(int16_t ch)
{
    static bool_t shoot_flag = 0;

    if (ch > CH4_HIGH_VALUE)
    {
        shoot_flag = 1;
    }
    else if (ch < CH4_LOW_VALUE)
    {
        shoot_flag = 0;
    }

    return shoot_flag;
}
// 只处理射击状态机
static void gimbal_shoot_set_control(gimbal_t *gimbal)
{
    static bool_t last_shoot_switch = 0;
    static bool_t last_fric_switch = 0;

    if (gimbal == NULL) return;

    gimbal_shoot_t *shoot = &gimbal->gimbal_shoot;
    const bool auto_aim_on = (gimbal->anto_aim_mode_cmd == true);

#ifdef USE_SBUS_PROTOCOL
    /* SBUS 三段开关逻辑保留；自瞄时只保证摩擦轮，不强制连发 */
    int8_t sw_val = gimbal->gimbal_RC->rc.s[RIGHT_1_SWITCH];

    if (auto_aim_on)
    {
        /* 自瞄模式：至少 READY_FRIC，不在此处自动进入 BULLET/CONTINUE */
        if (shoot->shoot_mode == SHOOT_STOP)
            shoot->shoot_mode = SHOOT_READY_FRIC;

        /* 允许手动触发：UP(+1) 时才给开火，MID 保持 READY */
        if (sbus_sw_is_up(sw_val) && shoot->shoot_mode >= SHOOT_READY)
            shoot->shoot_mode = SHOOT_CONTINUE_BULLET;
        else if (shoot->shoot_mode == SHOOT_CONTINUE_BULLET && !sbus_sw_is_up(sw_val))
            shoot->shoot_mode = SHOOT_READY;
    }
    else
    {
        if (sbus_sw_is_down(sw_val))
        {
            shoot->shoot_mode = SHOOT_STOP;
        }
        else
        {
            if (shoot->shoot_mode == SHOOT_STOP)
                shoot->shoot_mode = SHOOT_READY_FRIC;
            if (sbus_sw_is_up(sw_val) && shoot->shoot_mode >= SHOOT_READY)
                shoot->shoot_mode = SHOOT_CONTINUE_BULLET;
            else if (shoot->shoot_mode == SHOOT_CONTINUE_BULLET)
                shoot->shoot_mode = SHOOT_READY;
        }
    }

#else
    static bool_t shoot_press_latched = 0;
    static uint32_t shoot_press_tick = 0;

#ifdef USE_PC_CONTROL
    bool cur_fric_sw = switch_is_fric_on(gimbal->gimbal_PC_RC->rc.wheel) || gimbal->gimbal_PC_RC->mouse.press_r;
    bool cur_shoot_sw = switch_is_shoot(gimbal->gimbal_PC_RC->rc.wheel) || gimbal->gimbal_PC_RC->mouse.press_l;
#else
    bool_t cur_fric_sw = switch_is_fric_on(gimbal->gimbal_RC->rc.ch[RC_ROLL_CH]);
    bool_t cur_shoot_sw = switch_is_shoot(gimbal->gimbal_RC->rc.ch[RC_ROLL_CH]);
#endif
    uint32_t now_tick = osKernelSysTick();

    /* 自瞄模式下：强制摩擦轮开启，不允许“关摩擦轮”toggle */
    if (auto_aim_on)
    {
        if (shoot->shoot_mode == SHOOT_STOP)
            shoot->shoot_mode = SHOOT_READY_FRIC;
    }
    else
    {
        if (cur_fric_sw && !last_fric_switch)
            shoot->shoot_mode = (shoot->shoot_mode == SHOOT_STOP) ? SHOOT_READY_FRIC : SHOOT_STOP;
    }

    if (cur_shoot_sw)
    {
        if (!shoot_press_latched)
        {
            shoot_press_latched = 1;
            shoot_press_tick = now_tick;
        }
    }
    else
    {
        shoot_press_latched = 0;
        shoot_press_tick = now_tick;
    }

    /* 开火始终由手动输入触发（含自瞄模式） */
    if (shoot->shoot_mode == SHOOT_READY && cur_shoot_sw && !last_shoot_switch)
        shoot->shoot_mode = SHOOT_BULLET;

    if (cur_shoot_sw && shoot_press_latched &&
        (uint32_t)(now_tick - shoot_press_tick) >= SHOOT_CONTINUE_HOLD_MS &&
        shoot->shoot_mode >= SHOOT_READY)
        shoot->shoot_mode = SHOOT_CONTINUE_BULLET;

    if (shoot->shoot_mode == SHOOT_CONTINUE_BULLET && !cur_shoot_sw)
        shoot->shoot_mode = SHOOT_READY;

    last_fric_switch = cur_fric_sw;
    last_shoot_switch = cur_shoot_sw;
#endif

    if (shoot->shoot_mode == SHOOT_BULLET && shoot->bullet_done)
    {
        shoot->shoot_mode = SHOOT_READY;
        shoot->bullet_done = false;
    }

    if (shoot->shoot_mode == SHOOT_READY_FRIC &&
        fabsf(shoot->shoot_left_rad) > fabsf(SHOOT_SPD * 0.9f) &&
        fabsf(shoot->shoot_right_rad) > fabsf(SHOOT_SPD * 0.9f))
        shoot->shoot_mode = SHOOT_READY;

    shoot->shoot_target_speed = (shoot->shoot_mode == SHOOT_STOP) ? 0.0f : SHOOT_SPD;
}


//////////////////////////////////////////////////////////////////////
/**
 * @brief 设置云台控制量
 * @param gimbal_set_control 云台控制结构体指针
 * @details 根据当前工作模式设置云台目标位置和发射轮目标转速
 */
void gimbal_set_control(gimbal_t *gimbal_set_control)
{

    float yaw_set = 0, pitch_set = 0;

    // 无力模式：强制射击停止
    if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_STOP;
        return;
    }
    // 慢速校准模式：执行校准控制，射击停止
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
    {
        calibrate_control(gimbal_set_control, &yaw_set, &pitch_set);
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_STOP;
    }
    // 可手动射击模式（正常/云台跟随底盘/PC控制）
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NORMAL ||
             gimbal_set_control->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS ||
             gimbal_set_control->gimbal_mod == GIMBAL_MOD_CONTROL_BY_PC)
    {
        // 自动瞄准模式：强制启动摩擦轮，转速达标后自动进入连发
        if (gimbal_set_control->anto_aim_mode_cmd == true)
        {
            //自动控制位置
            minipc_control(gimbal_set_control, &yaw_set, &pitch_set);
        }
        else
        {
            // 非自动瞄准模式下，手动控制位置
        gimbal_rc_set_control(gimbal_set_control, &yaw_set, &pitch_set);
        }

    }
    //射击控制
    gimbal_shoot_set_control(gimbal_set_control);

    // 更新目标位置
    gimbal_set_control->gimbal_pos.last_pitch_target_pos = gimbal_set_control->gimbal_pos.pitch_target_pos;
    limit_pitch_command_by_encoder(gimbal_set_control, &pitch_set);
    gimbal_set_control->gimbal_pos.pitch_target_pos += pitch_set;

    // gimbal_set_control->gimbal_pos.pitch_target_pos = rad_format(gimbal_set_control->gimbal_pos.pitch_target_pos);
    // PITCH目标仍保留软限幅，真正机械保护由编码器硬限位执行
    limitPos(&gimbal_set_control->gimbal_pos.pitch_target_pos, MAX_PITCH_RAD, MIN_PITCH_RAD);

    gimbal_set_control->gimbal_pos.last_yaw_target_pos = gimbal_set_control->gimbal_pos.yaw_target_pos;
    gimbal_set_control->gimbal_pos.yaw_target_pos += yaw_set;
    // 跨越±pi时按当前反馈就近展开，避免目标值突变造成误差突跳
    cross_roattion_handle(&gimbal_set_control->gimbal_pos.yaw_target_pos,
                          &gimbal_set_control->gimbal_pos.yaw_relattive_pos);
}

/**
 * @brief 拨弹轮堵转反转处理
 * @param sh 发射系统结构体指针
 * @details 基于计数器检测持续慢速（堵转），自动切换反转方向，计时完成后复位
 */
static void trigger_motor_turn_back(gimbal_shoot_t *sh)
{
    // 未堵转时正向拨弹，堵转后反转解卡
    if (sh->block_count < BLOCK_TIME_COUNT)
        sh->pull_target_speed = SHOOT_PULL_FORWARD_SPEED;
    else
        sh->pull_target_speed = SHOOT_PULL_REVERSE_SPEED;

    if (fabsf(sh->shoot_pull_rad) < BLOCK_TRIGGER_SPEED && sh->block_count < BLOCK_TIME_COUNT)
    {
        // 速度持续低于阈值，累加堵转计数
        if (sh->reverse_count > 0)
            sh->reverse_count--; // 给反转恢复一段冷却窗口
        else
            sh->block_count++;
    }
    else if (sh->block_count >= BLOCK_TIME_COUNT && sh->reverse_count < REVERSE_TIME_COUNT)
    {
        sh->reverse_count++; // 反转计时
    }
    else
    {
        sh->block_count = 0; // 反转完成，复位堵转计数
    }

    old_PID_Calc(&sh->Pull_PID, sh->shoot_pull_rad, sh->pull_target_speed);
}

/**
 * @brief 计算云台发射系统控制量
 * @param sh 发射系统结构体指针
 * @details 根据射击状态机驱动摩擦轮和拨弹轮，包含预热检测与堵转处理
 */
void cal_gimbal_shoot_control(gimbal_shoot_t *shoot)
{
    static bool pull_ecd_inited = false;
    static int32_t pull_ecd_turn_cnt = 0;
    static uint16_t pull_last_ecd = 0;
    static bool single_shot_active = false;
    static fp32 single_shot_start_angle = 0.0f;
    static uint32_t single_shot_start_tick = 0;
    static bool continue_shot_active = false;
    static fp32 continue_shot_start_angle = 0.0f;

    const fp32 shot_direction_sign = (SHOOT_PULL_FORWARD_SPEED >= 0.0f) ? 1.0f : -1.0f;
    uint32_t now_tick = osKernelSysTick();

    uint16_t pull_cur_ecd = shoot->pull_motor->pos;
    if (!pull_ecd_inited)
    {
        pull_last_ecd = pull_cur_ecd;
        pull_ecd_inited = true;
    }
    else
    {
        int32_t ecd_delta = (int32_t)pull_cur_ecd - (int32_t)pull_last_ecd;
        if (ecd_delta > Half_ecd_range)
        {
            pull_ecd_turn_cnt--;
        }
        else if (ecd_delta < -Half_ecd_range)
        {
            pull_ecd_turn_cnt++;
        }
        pull_last_ecd = pull_cur_ecd;
    }

    fp32 pull_abs_angle = ((fp32)(pull_ecd_turn_cnt * ecd_range + (int32_t)pull_cur_ecd)) * Motor_Ecd_to_Rad;

    // ——— 摩擦轮 ———
    if (shoot->shoot_mode == SHOOT_STOP)
    {
        // 减速制动到零
        old_PID_Calc(&shoot->Shoot_left_PID, shoot->shoot_left_rad, 0.0f);
        old_PID_Calc(&shoot->Shoot_right_PID, shoot->shoot_right_rad, 0.0f);
        if (fabsf(shoot->shoot_left_rad) < 80.0f && fabsf(shoot->shoot_right_rad) < 80.0f)
        {
            old_PID_clear(&shoot->Shoot_left_PID);
            old_PID_clear(&shoot->Shoot_right_PID);
            old_PID_clear(&shoot->Pull_PID);
            shoot->shoot_motor_right->target_current = 0;
            shoot->shoot_motor_left->target_current = 0;
        }
        else
        {
            shoot->shoot_motor_right->target_current = (int16_t)shoot->Shoot_right_PID.out;
            shoot->shoot_motor_left->target_current = (int16_t)shoot->Shoot_left_PID.out;
        }
        shoot->pull_motor->target_current = 0;
        shoot->block_count = 0;
        shoot->reverse_count = 0;
        single_shot_active = false;
        continue_shot_active = false;
        return;
    }

    // 低速时使用锁定Kp提供更大启动力矩
    if (fabsf(shoot->shoot_left_rad) < fabsf(SHOOT_SPD * 0.5f) && fabsf(shoot->shoot_right_rad) < fabsf(SHOOT_SPD * 0.5f))
    {
        shoot->Shoot_right_PID.Kp = PID_SHOOT_LOCK_KP;
        shoot->Shoot_left_PID.Kp = PID_SHOOT_LOCK_KP;
    }
    else
    {
        shoot->Shoot_right_PID.Kp = PID_SHOOT_RIGHT_MOTOR_KP;
        shoot->Shoot_left_PID.Kp = PID_SHOOT_LEFT_MOTOR_KP;
    }
    old_PID_Calc(&shoot->Shoot_left_PID, shoot->shoot_left_rad, shoot->shoot_target_speed);
    old_PID_Calc(&shoot->Shoot_right_PID, shoot->shoot_right_rad, shoot->shoot_target_speed);
    shoot->shoot_motor_right->target_current = (int16_t)shoot->Shoot_right_PID.out;
    shoot->shoot_motor_left->target_current = (int16_t)shoot->Shoot_left_PID.out;

    // ——— 拨弹轮 ———
    if (shoot->shoot_mode <= SHOOT_READY)
    {
        // 预热或待机阶段：拨弹轮不转，重置堵转计数
        shoot->pull_motor->target_current = 0;
        shoot->block_count = 0;
        shoot->reverse_count = 0;
        single_shot_active = false;
        continue_shot_active = false;
        return;
    }

    if (shoot->shoot_mode == SHOOT_BULLET)
    {
        // 单发模式：按固定方向累计拨弹角度，达到 3*pi 后判定一发完成
        continue_shot_active = false;
        shoot->pull_target_speed = SHOOT_PULL_FORWARD_SPEED;
        old_PID_Calc(&shoot->Pull_PID, shoot->shoot_pull_rad, shoot->pull_target_speed);
        shoot->pull_motor->target_current = (int16_t)shoot->Pull_PID.out;

        if (!single_shot_active)
        {
            single_shot_start_angle = pull_abs_angle;
            single_shot_start_tick = now_tick;
            single_shot_active = true;
        }

        fp32 single_shot_progress = fabsf((pull_abs_angle - single_shot_start_angle) * shot_direction_sign);
        bool single_shot_timeout = ((uint32_t)(now_tick - single_shot_start_tick) >= SHOOT_SINGLE_TIMEOUT_MS);

        if (single_shot_progress >= SHOOT_SINGLE_BULLET_RAD || single_shot_timeout)
        {
            shoot->bullet_done = true;
            single_shot_active = false;
            shoot->block_count = 0;
            shoot->reverse_count = 0;
        }
    }
    else
    {
        // 连发模式：按 3*pi 角度步进判定每一发，保留堵转反转处理
        single_shot_active = false;

        if (!continue_shot_active)
        {
            continue_shot_start_angle = pull_abs_angle;
            continue_shot_active = true;
        }

        fp32 continue_shot_progress = (pull_abs_angle - continue_shot_start_angle) * shot_direction_sign;
        if (continue_shot_progress >= SHOOT_SINGLE_BULLET_RAD)
        {
            // 达到一发角度后推进下一发目标角，形成稳定节拍
            continue_shot_start_angle += SHOOT_SINGLE_BULLET_RAD * shot_direction_sign;
            shoot->block_count = 0;
            shoot->reverse_count = 0;
        }

        trigger_motor_turn_back(shoot);
        shoot->pull_motor->target_current = (int16_t)shoot->Pull_PID.out;
    }
}

/**
 * @brief 云台控制主循环
 * @param gimbal_loop 云台控制结构体指针
 * @details 执行云台位置PID控制和发射系统控制，包含PITCH和YAW轴的串级PID控制
 */
void gimbal_control_loop(gimbal_t *gimbal_loop)
{
    if (gimbal_loop == NULL)
    {
        return;
    }

    // 获取底盘角速度反馈（用于小陀螺模式和跟随底盘模式补偿）
    float chassis_yaw_rate = 0.0f;
    const chassis_feedback_frame_t *chassis_fb = gimbal_get_feedback();
    if (chassis_fb != NULL) {
        chassis_yaw_rate = chassis_fb->yaw_rate / 1000.0f; // 转换为rad/s
    }

    // ==================== 无力模式控制 ====================
    if (gimbal_loop->gimbal_mod == GIMBAL_MOD_NO_FORCE)
    {
        gimbal_no_force_control(gimbal_loop);
    }
    else
    {
        // ==================== PITCH轴控制 ====================
        // 所有非无力模式都需要PITCH轴控制
        gimbal_pitch_control(gimbal_loop);

        // ==================== YAW轴控制（根据模式选择） ====================
        if (gimbal_loop->gimbal_mod == GIMBAL_MOD_SLOW_CALI)
        {
            // 慢速校准模式
            gimbal_slow_cali_control(gimbal_loop);
        }
        else if (gimbal_loop->spinning_mode)
        {
            // 小陀螺模式
            gimbal_spinning_mode_control(gimbal_loop, chassis_yaw_rate);
        }
        else if (gimbal_loop->gimbal_mod == GIMBAL_MOD_FLOW_CHASSIS)
        {
            // 云台跟随底盘模式
            gimbal_follow_chassis_control(gimbal_loop, chassis_yaw_rate);
        }
        else
        {
            // 正常模式（包括GIMBAL_MOD_NORMAL、GIMBAL_MOD_AutoAim、GIMBAL_MOD_CONTROL_BY_PC）
            gimbal_normal_mode_control(gimbal_loop);
        }
    }

    // ==================== 发射系统控制 ====================
    // 控制发射系统（状态机驱动，包含摩擦轮预热、拨弹轮堵转处理）
    cal_gimbal_shoot_control(&gimbal_loop->gimbal_shoot);
}
/* gimbal_ctl_chassis_cmd 已移至 ctl_chassis.c */

/**
 * @brief 获取云台控制结构体只读指针
 * @return 指向内部 static gimbal 结构体的只读指针
 */
const gimbal_t *get_gimbal_point(void)
{
    return &gimbal;
}

bool is_gimbal_init_done(void)
{
    return gimbal_init_done;
}

/**
 * @brief 更新云台小陀螺模式状态
 * @param gimbal 云台控制结构体指针
 * @details 在小陀螺模式下，云台切换为绝对角度控制，并补偿底盘旋转
 */
void update_gimbal_spinning_mode(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;

    // 同步小陀螺指令状态
    gimbal->spinning_mode_cmd = gimbal->chassis_cmd.spinning_cmd;

    // 更新小陀螺模式状态
    bool old_spinning_mode = gimbal->spinning_mode;
    gimbal->spinning_mode = gimbal->chassis_cmd.spinning_cmd;

    // 检测小陀螺模式切换（上升沿）
    if (gimbal->spinning_mode && !old_spinning_mode)
    {
        // 切换到小陀螺模式：将目标位置锁定为当前绝对角度
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.yaw_absolute_pos;
        gimbal->gimbal_pos.pitch_target_pos = gimbal->gimbal_pos.pitch_absolute_pos;

        // 清空PID积分器，防止切换时的积分累积
        old_PID_clear(&gimbal->gimbal_pos.yaw_absolute_angle_pid);
        old_PID_clear(&gimbal->gimbal_pos.yaw_motor_gyro_pid);
        old_PID_clear(&gimbal->gimbal_pos.yaw_relative_angle_pid);

        // 更新状态记录
        gimbal->last_spinning_mode_cmd = gimbal->spinning_mode_cmd;
    }
    // 检测小陀螺模式退出（下降沿）
    else if (!gimbal->spinning_mode && old_spinning_mode)
    {
        // 退出小陀螺模式：将目标位置切换到当前位置，避免跳变
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.yaw_relattive_pos;
        gimbal->gimbal_pos.pitch_target_pos = gimbal->gimbal_pos.pitch_absolute_pos;

        // 清空PID积分器
        old_PID_clear(&gimbal->gimbal_pos.yaw_relative_angle_pid);
        old_PID_clear(&gimbal->gimbal_pos.yaw_motor_gyro_pid);

        // 更新状态记录
        gimbal->last_spinning_mode_cmd = gimbal->spinning_mode_cmd;
    }
}

/* ==================== 新增控制函数实现 ==================== */

/**
 * @brief 无力模式控制
 * @param gimbal 云台控制结构体指针
 * @details 在无力模式下，所有电机输出清零
 */
static void gimbal_no_force_control(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;
    
    gimbal->gimbal_pos.pitch_motor_measure->target_tor = 0;
    gimbal->gimbal_pos.yaw_motor_measure->target_current = 0;
    gimbal->gimbal_shoot.shoot_motor_right->target_current = 0;
    gimbal->gimbal_shoot.shoot_motor_left->target_current = 0;
    gimbal->gimbal_shoot.pull_motor->target_current = 0;
}

/**
 * @brief PITCH轴控制
 * @param gimbal 云台控制结构体指针
 * @details 执行PITCH轴角度环PID控制，包含机械限位保护
 */
static void gimbal_pitch_control(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;
    
    // PITCH轴角度环PID控制
    gimbal->gimbal_pos.pitch_motor_measure->target_tor =
        old_PID_Calc(&gimbal->gimbal_pos.gimbal_motor_absolute_angle_pid,
                     gimbal->gimbal_pos.pitch_absolute_pos,
                     gimbal->gimbal_pos.pitch_target_pos);

    // 机械限位保护：当达到限位且目标方向会进一步超出限位时，停止输出
    if ((gimbal->gimbal_pos.pitch_relatiive_pos >= MAX_PITCH_RAD &&
         gimbal->gimbal_pos.pitch_target_pos > gimbal->gimbal_pos.pitch_absolute_pos) ||
        (gimbal->gimbal_pos.pitch_relatiive_pos <= MIN_PITCH_RAD &&
         gimbal->gimbal_pos.pitch_target_pos < gimbal->gimbal_pos.pitch_absolute_pos))
    {
        gimbal->gimbal_pos.pitch_motor_measure->target_tor = 0.0f;
        gimbal->gimbal_pos.pitch_target_pos = gimbal->gimbal_pos.pitch_absolute_pos;
    }
}

/**
 * @brief 慢速校准模式控制
 * @param gimbal 云台控制结构体指针
 * @details 在校准模式下，使用相对角度环控制YAW轴归零
 */
static void gimbal_slow_cali_control(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;
    
    fp32 yaw_angle_error = gimbal->gimbal_pos.yaw_target_pos - gimbal->gimbal_pos.yaw_relattive_pos;
    yaw_angle_error = loop_float_constrain(yaw_angle_error, -PI, PI);

    fp32 yaw_motor_gyro_set = old_PID_Calc(&gimbal->gimbal_pos.yaw_relative_angle_pid,
                                           0.0f,
                                           yaw_angle_error);

    // YAW轴速度环PID，输出作为电机扭矩
    gimbal->gimbal_pos.yaw_motor_measure->target_current =
        old_PID_Calc(&gimbal->gimbal_pos.yaw_motor_gyro_pid,
                     gimbal->gimbal_pos.yaw_motor_measure->real_w,
                     yaw_motor_gyro_set);
}

/**
 * @brief 小陀螺模式控制
 * @param gimbal 云台控制结构体指针
 * @param chassis_yaw_rate 底盘角速度 (rad/s)
 * @details 在小陀螺模式下，保持云台在世界坐标系中的绝对角度不变
 */
static void gimbal_spinning_mode_control(gimbal_t *gimbal, float chassis_yaw_rate)
{
    if (gimbal == NULL) return;
    
    // 计算绝对角度误差（世界坐标系）
    fp32 yaw_abs_error = gimbal->gimbal_pos.yaw_target_pos - gimbal->gimbal_pos.yaw_absolute_pos;
    yaw_abs_error = loop_float_constrain(yaw_abs_error, -PI, PI);

    // 死区限制，防止小误差震荡
    if (fabsf(yaw_abs_error) < 0.01f)
    {
        yaw_abs_error = 0.0f;
    }

    // 绝对角度环PID，输出作为速度环的目标值
    fp32 yaw_motor_gyro_set = old_PID_Calc(&gimbal->gimbal_pos.yaw_absolute_angle_pid,
                                           0.0f,
                                           yaw_abs_error);

    // 添加底盘角速度前馈补偿：云台需要反向旋转以抵消底盘旋转
    // 底盘顺时针转(正角速度) → 云台需要逆时针转(负角速度)
    float feedforward_gain = 1.0f; // 前馈增益，可调节
    yaw_motor_gyro_set -= chassis_yaw_rate * feedforward_gain;

    // 限制最大速度设定值
    const float MAX_GYRO_SET = 10.0f; // rad/s
    if (yaw_motor_gyro_set > MAX_GYRO_SET) yaw_motor_gyro_set = MAX_GYRO_SET;
    if (yaw_motor_gyro_set < -MAX_GYRO_SET) yaw_motor_gyro_set = -MAX_GYRO_SET;

    // YAW轴速度环PID
    gimbal->gimbal_pos.yaw_motor_measure->target_current =
        old_PID_Calc(&gimbal->gimbal_pos.yaw_motor_gyro_pid,
                     gimbal->ins_info->yaw_gyro,
                     yaw_motor_gyro_set);
}

/**
 * @brief 云台跟随底盘模式控制
 * @param gimbal 云台控制结构体指针
 * @param chassis_yaw_rate 底盘角速度 (rad/s)
 * @details 在跟随底盘模式下，云台保持相对底盘位置不变
 */
static void gimbal_follow_chassis_control(gimbal_t *gimbal, float chassis_yaw_rate)
{
    if (gimbal == NULL) return;
    
    // 计算相对角度误差
    fp32 yaw_rel_error = 0 - gimbal->gimbal_pos.yaw_relattive_pos;
    yaw_rel_error = loop_float_constrain(yaw_rel_error, -PI, PI);

    // 死区限制，防止小误差震荡
    if (fabsf(yaw_rel_error) < 0.01f)
    {
        yaw_rel_error = 0.0f;
    }

    // 相对角度环PID，输出作为速度环的目标值
    fp32 yaw_motor_gyro_set = old_PID_Calc(&gimbal->gimbal_pos.yaw_relative_angle_pid,
                                           0.0f,
                                           yaw_rel_error);

    // 添加底盘角速度前馈补偿，抵消底盘旋转带来的干扰
    float feedforward_gain = 0.2f; // 前馈增益，需要根据实际系统调整
    yaw_motor_gyro_set += chassis_yaw_rate * feedforward_gain;

    // 限制最大速度设定值
    const float MAX_GYRO_SET = 8.0f; // rad/s
    if (yaw_motor_gyro_set > MAX_GYRO_SET) yaw_motor_gyro_set = MAX_GYRO_SET;
    if (yaw_motor_gyro_set < -MAX_GYRO_SET) yaw_motor_gyro_set = -MAX_GYRO_SET;

    // YAW轴速度环PID
    gimbal->gimbal_pos.yaw_motor_measure->target_current =
        old_PID_Calc(&gimbal->gimbal_pos.yaw_motor_gyro_pid,
                     gimbal->ins_info->yaw_gyro,
                     yaw_motor_gyro_set);
}

/**
 * @brief 正常模式控制
 * @param gimbal 云台控制结构体指针
 * @details 在正常模式下，使用绝对角度环控制云台
 */
static void gimbal_normal_mode_control(gimbal_t *gimbal)
{
    if (gimbal == NULL) return;
    
    fp32 yaw_angle_error = gimbal->gimbal_pos.yaw_target_pos - gimbal->gimbal_pos.yaw_absolute_pos;
    // fp32 yaw_angle_error = gimbal->gimbal_pos.yaw_target_pos - gimbal->gimbal_pos.yaw_relattive_pos;
    yaw_angle_error = loop_float_constrain(yaw_angle_error, -PI, PI);

    // 死区限制，防止小误差震荡
    if (fabsf(yaw_angle_error) < 0.01f)
    {
        yaw_angle_error = 0.0f;
    }

    // 使用 (0, error) 方式调用PID，避免Target和Feedback跳变导致的微分震荡
    fp32 yaw_motor_gyro_set = old_PID_Calc(&gimbal->gimbal_pos.yaw_absolute_angle_pid,
                                           0.0f,
                                           yaw_angle_error);

    // YAW轴速度环PID
    gimbal->gimbal_pos.yaw_motor_measure->target_current =
        old_PID_Calc(&gimbal->gimbal_pos.yaw_motor_gyro_pid,
                     gimbal->ins_info->yaw_gyro,
                     yaw_motor_gyro_set);
}
