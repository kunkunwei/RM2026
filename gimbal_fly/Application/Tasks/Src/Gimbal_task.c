/**
 * @file Gimbal_task.c
 * @brief 云台控制任务源文件
 * @details 实现云台PID控制、模式切换、发射系统控制等功能
 * @author RM Team
 * @date 2025
 */

#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"


/**
 * @brief 遥控器死区限制宏
 * @details 对遥控器输入值进行死区处理，小于死区值时输出为0
 * @param input 输入值
 * @param output 输出值
 * @param dealine 死区阈值
 */
#define rc_deadline_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }

/* 全局变量定义 */
static gimbal_t gimbal;           // 云台控制结构体实例
extern uint8_t Usart_Mode; // 串口控制模式

/* 函数声明 */
void init_gimbal_control(gimbal_t *gimbal);                      // 云台控制初始化
void gimbal_set_mod(gimbal_t *gimbal_set_mod);                   // 设置云台工作模式
void gimbal_mod_change_date_transfer(gimbal_t *gimbal_transfer); // 模式切换数据转换
void gimbal_feedback(gimbal_t *gimbal_feedback);                 // 云台反馈数据更新
void gimbal_set_control(gimbal_t *gimbal_set_control);           // 设置云台控制量
void gimbal_control_loop(gimbal_t *gimbal_loop);                 // 云台控制循环
/**
 * @brief 云台控制任务主函数
 * @param argument FreeRTOS任务参数
 * @details 云台控制任务的主循环，负责模式设置、数据更新、控制计算等
 */
void Gimbal_Task(void const *argument)
{
    /* 等待系统初始化完成 */
    osDelay(500);
    TickType_t systick = 0;

    // 初始化云台控制系统
    init_gimbal_control(&gimbal);

    /* 无限循环 */
    for (;;) {
        systick = osKernelSysTick();

        gimbal_set_mod(&gimbal);                  // 设置云台工作模式
        gimbal_mod_change_date_transfer(&gimbal); // 模式切换时的数据转换
        gimbal_feedback(&gimbal);                 // 更新反馈数据
        gimbal_set_control(&gimbal);              // 设置控制目标
        gimbal_control_loop(&gimbal);             // 执行控制循环

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
    shoot->reverse_count = 0;                // 初始目标转速为0

    // 初始化堵转检测时间
    // shoot->no_block_update_time = xTaskGetTickCount();
    // shoot->block_time           = xTaskGetTickCount();
}

/**
 * @brief 初始化云台位置控制系统
 * @param pos_control 位置控制结构体指针
 * @details 初始化滤波器、PID控制器、获取电机指针、设置初始状态
 */
void init_gimbal_pos_control(gimbal_pos_control_t *pos_control)
{

    // 滤波器参数设置
    const static fp32 gimbal_yaw_order_filter[1]   = {0.166f}; // YAW轴遥控滤波器参数
    const static fp32 gimbal_pitch_order_filter[1] = {0.166f}; // PITCH轴遥控滤波器参数
    const static fp32 gimbal_roll_order_filter[1] = {0.166f}; // ROLL轴遥控滤波器参数
    const static fp32 gimbal_yaw_pos_filter[1]     = {0.2f};   // YAW轴位置滤波器参数
    const static fp32 gimbal_pitch_pos_filter[1]   = {0.2f};   // PITCH轴位置滤波器参数
    const static fp32 gimbal_roll_pos_filter[1]   = {0.2f};   // ROLL轴位置滤波器参数

    // 初始化一阶低通滤波器
    first_order_filter_init(&pos_control->filter_rc_yaw_vel_set, 0.002f, gimbal_yaw_order_filter);     // 遥控YAW速度滤波器
    first_order_filter_init(&pos_control->filter_rc_pitch_vel_set, 0.002f, gimbal_pitch_order_filter); // 遥控PITCH速度滤波器
    first_order_filter_init(&pos_control->filter_rc_roll_vel_set, 0.002f, gimbal_roll_order_filter); // 遥控ROLL速度滤波器
    first_order_filter_init(&pos_control->filter_MiniPc_pitch_set, 0.002, gimbal_pitch_pos_filter);    // 小电脑PITCH位置滤波器
    first_order_filter_init(&pos_control->filter_MiniPc_yaw_set, 0.002, gimbal_yaw_pos_filter);        // 小电脑YAW位置滤波器
    first_order_filter_init(&pos_control->filter_MiniPc_roll_set, 0.002, gimbal_roll_pos_filter);        // 小电脑ROLL位置滤波器

    // 初始化PITCH轴PID控制器
    static const fp32 Pitch_speed_pid[3]    = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};                         // 速度环PID参数
    static const fp32 Pitch_absolute_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->gimbal_motor_absolute_angle_pid, PID_POSITION, Pitch_absolute_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    // 初始化YAW轴PID控制器
    static const fp32 Yaw_speed_pid[3]    = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};                         // 速度环PID参数
    static const fp32 Yaw_absolute_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->yaw_absolute_angle_pid, PID_POSITION, Yaw_absolute_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->yaw_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    // 初始化ROLL轴PID控制器
    static const fp32 Roll_speed_pid[3]    = {ROLL_SPEED_PID_KP, ROLL_SPEED_PID_KI, ROLL_SPEED_PID_KD};                         // 速度环PID参数
    static const fp32 Roll_absolute_pid[3] = {ROLL_GYRO_ABSOLUTE_PID_KP, ROLL_GYRO_ABSOLUTE_PID_KI, ROLL_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->roll_absolute_angle_pid, PID_POSITION, Roll_absolute_pid, ROLL_GYRO_ABSOLUTE_PID_MAX_OUT, ROLL_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->roll_motor_gyro_pid, PID_POSITION, Roll_speed_pid, ROLL_SPEED_PID_MAX_OUT, ROLL_SPEED_PID_MAX_IOUT);

    // 初始化目标位置
    pos_control->yaw_target_pos   = 0.0f;
    pos_control->pitch_target_pos = 0.0f;
    pos_control->roll_target_pos = 0.0f;

    // 获取电机测量数据指针
    pos_control->pitch_motor_measure = get_pitch_motor();
    pos_control->yaw_motor_measure   = get_yaw_motor();
    pos_control->roll_motor_measure   = get_roll_motor();

    // 初始化状态标志
    pos_control->calibrate_warning = false;

    // 设置初始电流为0，初始化相对位置
    pos_control->pitch_motor_measure->target_tor = 0;
    pos_control->yaw_relattive_pos                   = 0.0f;
    pos_control->roll_relattive_pos                   = 0.0f;

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
    gimbal->gimbal_RC   = get_sbus_remote_control_point(); // 遥控器数据
#else
    gimbal->gimbal_RC   = get_remote_control_point(); // 遥控器数据
#endif
    gimbal->gimbal_mod  = GIMBAL_MOD_NO_FORCE;        // 初始模式为无力
    gimbal->minipc_info = getUsbDpkgData();           // 小电脑通信数据
    gimbal->ins_info    = get_ins_info_point();       // 惯性导航数据

    // 更新反馈数据并设置初始位置
    gimbal_feedback(gimbal);
    gimbal->gimbal_pos.pitch_target_pos = gimbal->ins_info->pit_angle; // PITCH目标位置为当前角度
    gimbal->gimbal_pos.init_pitch_pos   = gimbal->ins_info->pit_angle; // 记录PITCH初始位置

    gimbal->gimbal_pos.roll_target_pos = -gimbal->ins_info->rol_angle; // ROLL目标位置为当前角度
    gimbal->gimbal_pos.init_roll_pos   = -gimbal->ins_info->rol_angle; // 记录ROLL初始位置

    // 初始化其他状态变量
    gimbal->gimbal_pos.yaw_relattive_pos   = 0.0f;                      // YAW相对位置清零
    // gimbal->gimbal_shoot.pull_target_speed = SHOOT_PULL_TOWARD_POSTIVE; // 拨弹轮初始目标速度
    // gimbal->gimbal_shoot.pull_is_block     = false;                     // 拨弹轮无堵塞
    gimbal->gimbal_pos.add_pitch           = 0.0f;                      // PITCH增量清零
    gimbal->gimbal_pos.add_yaw             = 0.0f;                      // YAW增量清零
    gimbal->gimbal_pos.add_roll             = 0.0f;                      // ROLL增量清零


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
    // 遥控器控制模式
    gimbal_set_mod->last_gimbal_mod = gimbal_set_mod->gimbal_mod;
#ifdef USE_SBUS_PROTOCOL
    //左1拨杆上拨：无力模式
    if (gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]==-1)
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    // 左1拨杆下拨且当前为无力模式：进入慢速校准模式
    else if ((gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]==1) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time                   = xTaskGetTickCount();
    }
    // 左1拨杆下拨且当前为慢速校准模式：校准2秒后进入正常模式
    else if ((gimbal_set_mod->gimbal_RC->rc.s[LEFT_1_SWITCH]==1) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        if (xTaskGetTickCount() - cal_time < 2000) {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        } else {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 左2拨杆上拨且当前为自瞄模式：退出自瞄，进入正常模式
    else if ((gimbal_set_mod->gimbal_RC->rc.s[LEFT_2_SWITCH]==-1) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_AutoAim) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    // 左2拨杆下拨：自动瞄准模式
    else if ((gimbal_set_mod->gimbal_RC->rc.s[LEFT_2_SWITCH]==1)) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoAim;
    }
    //////////////////////////////////////////////////////////////////////
    if ((gimbal_set_mod->gimbal_RC->rc.s[RIGHT_2_SWITCH]==0)&&gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NORMAL) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoBalance;
    }
    else if ((gimbal_set_mod->gimbal_RC->rc.s[RIGHT_2_SWITCH]==-1)&&gimbal_set_mod->gimbal_mod != GIMBAL_MOD_NORMAL
        &&gimbal_set_mod->gimbal_mod != GIMBAL_MOD_NO_FORCE&&gimbal_set_mod->gimbal_mod != GIMBAL_MOD_SLOW_CALI) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }

    //////////////////////////////////////////////////////////////////////
#else

    // 右拨杆下拨：无力模式
    if (switch_is_down(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]))
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    // 右拨杆中位且当前为无力模式：进入慢速校准模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time                   = xTaskGetTickCount();
    }
    // 右拨杆中位且当前为慢速校准模式：校准2秒后进入正常模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        if (xTaskGetTickCount() - cal_time < 2000) {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        } else {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 右拨杆中位且当前为自瞄模式：退出自瞄，进入正常模式
    else if (switch_is_mid(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH]) && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_AutoAim) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    // 右拨杆上拨：自动瞄准模式
    else if (switch_is_up(gimbal_set_mod->gimbal_RC->rc.s[RIGHT_SWITCH])) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoAim;
    }
#endif
#endif

#ifdef USE_USART_CONTROL
    // 串口控制模式
    gimbal_set_mod->last_gimbal_mod = gimbal_set_mod->gimbal_mod;

    // 串口模式值越界检查
    if (Usart_Mode > 5) {
        Usart_Mode = 0;
        return;
    }

    // 串口模式0：无力模式
    if (Usart_Mode == 0) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NO_FORCE;
    }
    // 串口模式1且当前为无力模式：进入慢速校准模式
    else if (Usart_Mode == 1 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI;
        cal_time                   = xTaskGetTickCount();
    }
    // 串口模式1且当前为慢速校准模式：校准2秒后进入正常模式
    else if (Usart_Mode == 1 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        if (xTaskGetTickCount() - cal_time < 2000) {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_SLOW_CALI; // 继续校准
        } else {
            gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL; // 校准完成，进入正常模式
        }
    }
    // 串口模式1且当前为自瞄模式：退出自瞄，进入正常模式
    else if (Usart_Mode == 1 && gimbal_set_mod->gimbal_mod == GIMBAL_MOD_AutoAim) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_NORMAL;
    }
    // 串口模式2：自动瞄准模式
    else if (Usart_Mode == 2) {
        gimbal_set_mod->gimbal_mod = GIMBAL_MOD_AutoAim;
    }
#endif
}

/**
 * @brief 云台模式切换时的数据转换处理
 * @param gimbal_transfer 云台控制结构体指针
 * @details 在不同工作模式之间切换时，进行必要的数据转换和目标位置设置
 */
void gimbal_mod_change_date_transfer(gimbal_t *gimbal_transfer)
{
    // 无力模式：目标位置跟随当前位置
    if (gimbal_transfer->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->ins_info->pit_angle;
        gimbal_transfer->gimbal_pos.init_pitch_pos   = gimbal_transfer->ins_info->pit_angle;

        gimbal_transfer->gimbal_pos.roll_target_pos = gimbal_transfer->ins_info->rol_angle;
        gimbal_transfer->gimbal_pos.init_roll_pos   = gimbal_transfer->ins_info->rol_angle;

    }
    // 从无力模式切换到慢速校准模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_NO_FORCE && gimbal_transfer->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->ins_info->pit_angle;
        gimbal_transfer->gimbal_pos.init_pitch_pos   = gimbal_transfer->ins_info->pit_angle;

        gimbal_transfer->gimbal_pos.roll_target_pos = gimbal_transfer->ins_info->rol_angle;
        gimbal_transfer->gimbal_pos.init_roll_pos   = gimbal_transfer->ins_info->rol_angle;
    }
    // 从慢速校准模式切换到正常控制模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_SLOW_CALI && gimbal_transfer->gimbal_mod == GIMBAL_MOD_NORMAL) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.roll_target_pos   = gimbal_transfer->gimbal_pos.roll_absolute_pos;
    }
    // 切换到自动瞄准模式
    else if (gimbal_transfer->last_gimbal_mod != GIMBAL_MOD_AutoAim && gimbal_transfer->gimbal_mod == GIMBAL_MOD_AutoAim) {
        // 保留视觉目标设置的代码（已注释）
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.roll_target_pos   = gimbal_transfer->gimbal_pos.roll_absolute_pos;
    }
    //切换到自动平衡模式
    else if ( gimbal_transfer->last_gimbal_mod != GIMBAL_MOD_AutoBalance &&gimbal_transfer->gimbal_mod == GIMBAL_MOD_AutoBalance) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.roll_target_pos   = 0.0f;
    }
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
    gimbal_feedback->gimbal_pos.pitch_absolute_pos = rad_format(gimbal_feedback->gimbal_pos.pitch_absolute_pos);

    // 更新YAW轴绝对位置（来自IMU）
    gimbal_feedback->gimbal_pos.yaw_absolute_pos = gimbal_feedback->ins_info->yaw_angle;
    gimbal_feedback->gimbal_pos.yaw_absolute_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_absolute_pos);

    // 更新ROLL轴绝对位置（来自IMU）
    gimbal_feedback->gimbal_pos.roll_absolute_pos = -gimbal_feedback->ins_info->rol_angle;
    gimbal_feedback->gimbal_pos.roll_absolute_pos = rad_format(gimbal_feedback->gimbal_pos.roll_absolute_pos);

    // 更新YAW轴相对位置（来自电机编码器，考虑减速比）
    gimbal_feedback->gimbal_pos.yaw_relattive_pos += gimbal_feedback->gimbal_pos.yaw_motor_measure->deta_yaw_pos / REDDUCE_K;
    gimbal_feedback->gimbal_pos.yaw_relattive_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_relattive_pos);

    // 更新发射系统反馈数据（将RPM转换为rad/s）
    gimbal_feedback->gimbal_shoot.shoot_pull_rad  = gimbal_feedback->gimbal_shoot.pull_motor->rpm * 2.0f * PI / (36.0f * 60.0f);       // 拨弹轮（36:1减速比）
    gimbal_feedback->gimbal_shoot.shoot_left_rad  = -1.0f * gimbal_feedback->gimbal_shoot.shoot_motor_left->rpm * 2.0f * PI / (60.0f); // 左发射轮
    gimbal_feedback->gimbal_shoot.shoot_right_rad = gimbal_feedback->gimbal_shoot.shoot_motor_right->rpm * 2.0f * PI / (60.0f);        // 右发射轮
}
///////////////////////set mod end///////////////

/**
 * @brief 遥控器控制云台设置
 * @param gimbal 云台控制结构体指针
 * @param add_yaw YAW轴增量输出指针
 * @param add_pitch PITCH轴增量输出指针
 * @details 处理遥控器输入，经过死区限制和滤波后输出云台控制增量
 */
void gimbal_rc_set_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch, float *add_roll)
{
    if (gimbal == NULL) {
        return;
    }

    // 遥控器原始通道值
    int16_t wz_channel, pitch_channel,roll_channel;
    fp32 wz_set_channel, pitch_set_channel,roll_set_channel;
// #ifdef USE_SBUS_PROTOCOL
// #else
// #endif
    // 死区限制，避免遥控器摇杆中位时的微小偏差
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_LEFT_X_CH], wz_channel, 20);    // 左摇杆X轴控制YAW
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_X_CH], roll_channel, 20);    // 右摇杆X轴控制ROLL
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_Y_CH], pitch_channel, 20); // 右摇杆Y轴控制PITCH

    // 通道值转换为控制量（乘以灵敏度系数）
    wz_set_channel    = wz_channel * GIMBAL_WZ_RC_SEN;
    pitch_set_channel = pitch_channel * GIMBAL_PITCH_RC_SEN;
    roll_set_channel = roll_channel * GIMBAL_ROLL_RC_SEN;

    // 一阶低通滤波，平滑控制输入
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_yaw_vel_set, wz_set_channel);
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_pitch_vel_set, pitch_set_channel);
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_roll_vel_set, roll_set_channel);

    // 输出滤波后的控制增量
    *add_pitch = gimbal->gimbal_pos.filter_rc_pitch_vel_set.out;
    *add_yaw   = gimbal->gimbal_pos.filter_rc_yaw_vel_set.out;
    *add_roll   = gimbal->gimbal_pos.filter_rc_roll_vel_set.out;
}

////////set control start//////////////
/**
 * @brief 云台校准控制
 * @param gimbal 云台控制结构体指针
 * @param add_yaw YAW轴增量输出指针
 * @param add_pitch PITCH轴增量输出指针
 * @details 在校准模式下，将云台缓慢移动到零位
 */

void calibrate_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch, float *add_roll)
{

    // 缓慢将PITCH轴移动到零位（除以500减慢速度，指数衰减）
    *add_pitch = 0.0f -gimbal->gimbal_pos.pitch_target_pos / 800.0f;
    // 缓慢将ROLL轴移动到零位（除以500减慢速度，指数衰减）
    *add_roll = 0.0f -gimbal->gimbal_pos.roll_target_pos / 800.0f;

    // 当PITCH角度接近零位时，直接设置目标为零
    if (fabs(gimbal->ins_info->pit_angle) < 0.04f) {
        *add_pitch = 0.0f - gimbal->gimbal_pos.pitch_target_pos;
    }
    // 当ROLL角度接近零位时，直接设置目标为零
    if (fabs(gimbal->ins_info->rol_angle) < 0.04f) {
        *add_roll = 0.0f - gimbal->gimbal_pos.roll_target_pos;
    }
}
void autoBalance_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch, float *add_roll)
{
    if (gimbal == NULL) {
        return;
    }
    gimbal_rc_set_control(gimbal,add_yaw,add_pitch,add_roll);
    // 缓慢将PITCH轴移动到零位（除以800减慢速度）
    // *add_pitch = 0.0f - gimbal->gimbal_pos.init_pitch_pos / 1000.0f;
    // 缓慢将ROLL轴移动到零位（除以800减慢速度）
    *add_roll = 0.0f - gimbal->gimbal_pos.init_roll_pos / 800.0f;

    // // 当PITCH角度接近零位时，直接设置目标为零
    // if (fabs(gimbal->ins_info->pit_angle) < 0.04f) {
    //     *add_pitch = 0.0f - gimbal->gimbal_pos.pitch_target_pos;
    // }
    // 当ROLL角度接近零位时，直接设置目标为零
    if (fabs(gimbal->ins_info->rol_angle) < 0.04f) {
        *add_roll = 0.0f - gimbal->gimbal_pos.roll_target_pos;
    }
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
    if (isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw)) {
        *add_pitch = 0.0f;
        *add_yaw   = 0.0f;
        // 直接设置YAW目标位置
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw;
    }
    // 只有PITCH轴目标有效的情况
    else if (isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch)) {
        *add_yaw   = 0;
        *add_pitch = 0.0f;
    }
    // YAW和PITCH目标都有效的情况
    else if (!isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw) && !isnan(gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch)) {
        // 直接设置YAW目标位置
        gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw;
        *add_yaw                          = 0.0f;

        // 计算PITCH轴增量，缓慢调整（除以200减慢速度）
        gimbal->gimbal_pos.add_pitch = gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch - gimbal->gimbal_pos.pitch_target_pos;
        *add_pitch                   = gimbal->gimbal_pos.add_pitch / 200.0f;
    }
    // 都无效的情况
    else {
        *add_yaw   = 0.0f;
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

/**
 * @brief 跨零点旋转处理
 * @param target_pos 目标位置指针
 * @param this_time_pos 当前位置指针
 * @details 处理角度跨越±π时的连续性问题，选择最短路径
 */
void cross_roattion_handle(float *target_pos, const float *this_time_pos)
{
    // 如果目标位置与当前位置差值过大，调整目标位置以选择最短路径
    if (*target_pos - *this_time_pos > 2.0f * PI / 3.0f) {
        *target_pos -= 2.0f * PI; // 目标位置减去2π
    } else if (*target_pos - *this_time_pos < -2.0f * PI / 3.0f) {
        *target_pos += 2.0f * PI; // 目标位置加上2π
    } else {
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
//////////////////////////////////////////////////////////////////////
/**
 * @brief 设置云台控制量
 * @param gimbal_set_control 云台控制结构体指针
 * @details 根据当前工作模式设置云台目标位置和发射轮目标转速
 */
void gimbal_set_control(gimbal_t *gimbal_set_control)
{
    static bool_t last_shoot_switch = 0;
    static bool_t last_fric_switch = 0;
    float yaw_set = 0, pitch_set = 0, roll_set = 0;

    // 无力模式：停止发射，直接返回
    if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_STOP;
        return;
    }
    // 慢速校准模式：执行校准控制，停止发射
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        calibrate_control(gimbal_set_control, &yaw_set, &pitch_set,&roll_set);
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_STOP;
    }
    // 正常控制模式：遥控器控制，停止发射
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NORMAL) {
#ifdef USE_RC_CONTROL
        gimbal_rc_set_control(gimbal_set_control, &yaw_set, &pitch_set,&roll_set);
        gimbal_shoot_t *shoot = &gimbal_set_control->gimbal_shoot;
#endif
        // gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
    }
    // 自动平衡模式：
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_AutoBalance) {
#ifdef USE_RC_CONTROL
        // gimbal_rc_set_control(gimbal_set_control, &yaw_set, &pitch_set,&roll_set);
        // roll_set=0.0f;
        autoBalance_control(gimbal_set_control, &yaw_set, &pitch_set,&roll_set);
#endif
        // gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
        gimbal_shoot_t *shoot = &gimbal_set_control->gimbal_shoot;
    }
    // 自动瞄准模式：视觉控制，启动发射轮
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_AutoAim) {
        minipc_control(gimbal_set_control, &yaw_set, &pitch_set);
        // gimbal_set_control->gimbal_shoot.shoot_target_speed = SHOOT_SPD;
        gimbal_shoot_t *shoot = &gimbal_set_control->gimbal_shoot;
    }
    //
    // if (gimbal_set_control->gimbal_RC->rc.s[RIGHT_1_SWITCH]==1)
    // {
    //     // gimbal_set_control->gimbal_shoot.shoot_target_speed = SHOOT_SPD;
    // }
    // else
    // {
    //     gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
    // }
    /* DBUS：通道 4 下拨切换摩擦轮，上拨短按单发、长按(>=1s)连发 */
    static bool_t shoot_press_latched = 0;
    static uint32_t shoot_press_tick = 0;
    bool_t cur_fric_sw = switch_is_fric_on(gimbal_set_control->gimbal_RC->rc.ch[RC_ROLL_CH]);
    // PC 图传当前按 wheel 统一控制摩擦轮/拨弹
    bool_t cur_shoot_sw = switch_is_shoot(gimbal_set_control->gimbal_RC->rc.ch[RC_ROLL_CH]);
    // 使用 RTOS systick 作为长按判定时基，避免 HAL tick 与任务时基不一致
    uint32_t now_tick = osKernelSysTick();
    // 摩擦轮 toggle（上升沿触发）
    if (cur_fric_sw && !last_fric_switch)
        gimbal_set_control->gimbal_shoot.shoot_mode = (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_STOP) ? SHOOT_READY_FRIC : SHOOT_STOP;

    // 按下时锁存时间戳，释放时清空
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

    // READY: 下降沿触发单发
    if (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_READY && cur_shoot_sw && !last_shoot_switch)
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_BULLET;
    // 长按升级连发：允许在 READY 下直接升级，避免单发完成后卡在 READY 无法进连发
    if (cur_shoot_sw && shoot_press_latched &&
        (uint32_t)(now_tick - shoot_press_tick) >= SHOOT_CONTINUE_HOLD_MS &&
        gimbal_set_control->gimbal_shoot.shoot_mode >= SHOOT_READY)
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_CONTINUE_BULLET;
    // 释放扔机退出连发
    if (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_CONTINUE_BULLET && !cur_shoot_sw)
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_READY;

    last_fric_switch = cur_fric_sw;
    last_shoot_switch = cur_shoot_sw;
    // 单发完成 → 返回 READY（bullet_done 由 shoot_bullet_control 置位）
    if (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_BULLET &&gimbal_set_control->gimbal_shoot.bullet_done)
    {
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_READY;
        gimbal_set_control->gimbal_shoot.bullet_done = false;
    }

    // READY_FRIC → READY：摩擦轮转速达到目标 90% 自动进入就绪
    if (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_READY_FRIC &&
        fabsf(gimbal_set_control->gimbal_shoot.shoot_left_rad) > fabsf(SHOOT_SPD * 0.9f) &&
        fabsf(gimbal_set_control->gimbal_shoot.shoot_right_rad) > fabsf(SHOOT_SPD * 0.9f))
        gimbal_set_control->gimbal_shoot.shoot_mode = SHOOT_READY;

    // 发射轮目标转速
    gimbal_set_control->gimbal_shoot.shoot_target_speed = (gimbal_set_control->gimbal_shoot.shoot_mode == SHOOT_STOP) ? 0.0f : SHOOT_SPD;
    // 更新目标位置
    gimbal_set_control->gimbal_pos.last_pitch_target_pos = gimbal_set_control->gimbal_pos.pitch_target_pos;
    gimbal_set_control->gimbal_pos.pitch_target_pos += pitch_set;

    gimbal_set_control->gimbal_pos.last_roll_target_pos = gimbal_set_control->gimbal_pos.roll_target_pos;
    gimbal_set_control->gimbal_pos.roll_target_pos += roll_set;
    // 限制PITCH轴角度范围
    limitPos(&gimbal_set_control->gimbal_pos.pitch_target_pos, MAX_PITCH_RAD, MIN_PITCH_RAD);
    // 限制ROLL轴角度范围
    limitPos(&gimbal_set_control->gimbal_pos.roll_target_pos, MAX_ROLL_RAD, MIN_ROLL_RAD);

    gimbal_set_control->gimbal_pos.last_yaw_target_pos = gimbal_set_control->gimbal_pos.yaw_target_pos;
    gimbal_set_control->gimbal_pos.yaw_target_pos += yaw_set;
    // 标准化YAW轴角度
    gimbal_set_control->gimbal_pos.yaw_target_pos = rad_format(gimbal_set_control->gimbal_pos.yaw_target_pos);

    // 处理跨零点旋转，选择最短路径
    // cross_roattion_handle(&gimbal_set_control->gimbal_pos.pitch_target_pos,
                          // &gimbal_set_control->gimbal_pos.pitch_absolute_pos);

    // cross_roattion_handle(&gimbal_set_control->gimbal_pos.yaw_target_pos,
    //                       &gimbal_set_control->gimbal_pos.yaw_absolute_pos);

    // cross_roattion_handle(&gimbal_set_control->gimbal_pos.roll_target_pos,
                          // &gimbal_set_control->gimbal_pos.roll_absolute_pos);
}

/**
 * @brief 检测拨弹轮是否卡弹
 * @param BlockDetect 发射系统结构体指针
 * @return 1-检测到卡弹, 0-卡弹解除, -2-状态无变化
 * @details 通过监测拨弹轮转速来判断是否发生卡弹，并提供防卡弹处理
 */
// int isShootBlock(gimbal_shoot_t *BlockDetect)
// {
//
//     // 拨弹轮正常转动时（速度小于-0.5），更新无堵转时间
//     if (BlockDetect->shoot_pull_rad < -0.5f) {
//         BlockDetect->no_block_update_time = xTaskGetTickCount();
//     }
//
//     // 卡弹反转1秒后，重置无堵转时间，准备重新检测
//     if (BlockDetect->pull_is_block == true && (xTaskGetTickCount() - BlockDetect->block_time > 1000)) {
//         BlockDetect->no_block_update_time = xTaskGetTickCount();
//         return 0; // 卡弹状态解除
//     }
//
//     // 持续1秒速度过小则判定为卡弹
//     if (BlockDetect->pull_is_block == false && (xTaskGetTickCount() - BlockDetect->no_block_update_time > 1000)) {
//         BlockDetect->block_time = xTaskGetTickCount();
//         return 1; // 检测到卡弹
//     }
//     return -2; // 状态无变化
// }
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
    if (gimbal_loop == NULL) {
        return;
    }

    // PITCH轴角度环PID控制
    gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor =
        old_PID_Calc(&gimbal_loop->gimbal_pos.gimbal_motor_absolute_angle_pid,
                              gimbal_loop->gimbal_pos.pitch_absolute_pos,
                              gimbal_loop->gimbal_pos.pitch_target_pos);
    // ROLL轴角度环PID控制
    gimbal_loop->gimbal_pos.roll_motor_measure->target_tor =
        old_PID_Calc(&gimbal_loop->gimbal_pos.roll_absolute_angle_pid,
                              gimbal_loop->gimbal_pos.roll_absolute_pos,
                              gimbal_loop->gimbal_pos.roll_target_pos);

    // 注释掉的速度环PID控制代码
    // gimbal_loop->gimbal_pos.pitch_motor_measure->target_current = \
    //             (int16_t)old_PID_Calc(&gimbal_loop->gimbal_pos.gimbal_motor_gyro_pid,\
    //                                  gimbal_loop->ins_info->pit_gyro,\
    //                                 gimbal_loop->gimbal_pos.tmp);

    // PITCH轴重力补偿（基于当前角度的前馈控制）
    // gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor -= (int16_t)gimbal_loop->gimbal_pos.pitch_absolute_pos * 15000.0f;
    gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor += (int16_t)gimbal_loop->gimbal_pos.pitch_absolute_pos * 15000.0f;
    gimbal_loop->gimbal_pos.roll_motor_measure->target_tor -= (int16_t)gimbal_loop->gimbal_pos.roll_absolute_pos * 1500.0f;

    // YAW轴双环PID控制（角度环+速度环），排除无力和校准模式
    if (gimbal_loop->gimbal_mod != GIMBAL_MOD_NO_FORCE && gimbal_loop->gimbal_mod != GIMBAL_MOD_SLOW_CALI) {
        // YAW轴角度环PID，输出作为速度环的目标值
        // gimbal_loop->gimbal_pos.yaw_motor_measure->target_tor = -old_PID_Calc(&gimbal_loop->gimbal_pos.yaw_absolute_angle_pid,
        //                                        gimbal_loop->gimbal_pos.yaw_absolute_pos,
        //                                        gimbal_loop->gimbal_pos.yaw_target_pos);
        fp32 yaw_motor_gyro_set = old_PID_Calc(&gimbal_loop->gimbal_pos.yaw_absolute_angle_pid,
                                               gimbal_loop->gimbal_pos.yaw_absolute_pos,
                                               gimbal_loop->gimbal_pos.yaw_target_pos);

        // YAW轴速度环PID，输出作为电机扭矩
        gimbal_loop->gimbal_pos.yaw_motor_measure->target_tor =
            old_PID_Calc(&gimbal_loop->gimbal_pos.yaw_motor_gyro_pid,
                         gimbal_loop->ins_info->yaw_gyro,
                         yaw_motor_gyro_set);
    }

    // 控制发射系统（发射轮常开，根据开火标志控制拨弹轮）
    // cal_gimbal_shoot_control(&gimbal_loop->gimbal_shoot,
    //                          true,
    //                          gimbal.gimbal_pos.usb_autoAim_ptr->shoot_flag);
    // 控制发射系统（状态机驱动，包含摩擦轮预热、拨弹轮堵转处理）
    cal_gimbal_shoot_control(&gimbal_loop->gimbal_shoot);
#ifdef USE_RC_CONTROL
#ifdef USE_SBUS_PROTOCOL
    // SBUS遥控器控制模式：左1拨杆下拨时停止所有电机
    if ((gimbal.gimbal_RC->rc.s[0]==-1)) {
        gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor = 0;
        gimbal_loop->gimbal_pos.yaw_motor_measure->target_tor    = 0;
        gimbal_loop->gimbal_pos.roll_motor_measure->target_tor    = 0;
    }
#else
    // DBUS遥控器控制模式：右拨杆下拨时停止所有电机
    if (switch_is_down(gimbal.gimbal_RC->rc.s[RIGHT_SWITCH])) {
        gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor = 0;
        gimbal_loop->gimbal_pos.yaw_motor_measure->target_tor    = 0;
        gimbal_loop->gimbal_pos.roll_motor_measure->target_tor    = 0;
    }
#endif

#else
    // 串口控制模式：无力模式时停止所有电机
    if (gimbal.gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_loop->gimbal_pos.pitch_motor_measure->target_current = 0;
        gimbal_loop->gimbal_pos.yaw_motor_measure->target_tor       = 0;
        gimbal_loop->gimbal_shoot.shoot_motor_right->target_current = 0;
        gimbal_loop->gimbal_shoot.shoot_motor_left->target_current  = 0;
        gimbal_loop->gimbal_shoot.pull_motor->target_current        = 0;
    }
#endif
}
const gimbal_t* get_gimbal_point(void)
{
    return &gimbal;
}