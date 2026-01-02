#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
#include "stepper_can.h"
#include "stepper_motor.h"
// #include "stepper_pwm_control.h"
#include "tim.h"
#include "usart.h"

//
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
gimbal_t gimbal;           // 云台控制结构体实例

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
void Gimbal_Task(void const * argument)
{
  /* Infinite loop */

    //一个周期是1us
    //100%占空比是1000


    osDelay(800);
    // osDelay(800);
    // 初始化云台控制系统
    init_gimbal_control(&gimbal);
    // StepperCAN_Enable( &yaw_motor_Info,ENABLE);
    // StepperCAN_Enable( &pitch_motor_Info,ENABLE);
    TickType_t systick = 0;
    // StepperCAN_SetSpeed(&yaw_motor_Info, 200,  100);

    for(;;)
    {
        // Stepper_Update();
        systick = osKernelSysTick();
        // StepperCAN_SetPositionAngle(&yaw_motor_Info,50, 20, 10, 1);
        // osDelay(2);
        // StepperCAN_SetPositionAngle(&pitch_motor_Info,50, 20, 10, 1);
        // StepperCAN_ReadSystemStatus(CAN1_YAW_MOTOR_ID);

        gimbal_set_mod(&gimbal);                  // 设置云台工作模式
        gimbal_mod_change_date_transfer(&gimbal); // 模式切换时的数据转换
        gimbal_feedback(&gimbal);                 // 更新反馈数据
        gimbal_set_control(&gimbal);              // 设置控制目标
        gimbal_control_loop(&gimbal);             // 执行控制循环

        osDelayUntil(&systick,2); // 2ms周期控制
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
    shoot->pull_motor         = get_shoot_motor_pull();  // 拨弹轮电机
    shoot->shoot_motor_right  = get_shoot_motor_right(); // 右发射轮电机
    shoot->shoot_motor_left   = get_shoot_motor_left();  // 左发射轮电机
    shoot->shoot_target_speed = 0.0f;                    // 初始目标转速为0

    // 初始化堵转检测时间
    shoot->no_block_update_time = xTaskGetTickCount();
    shoot->block_time           = xTaskGetTickCount();
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

    const static fp32 gimbal_yaw_pos_filter[1]     = {0.2f};   // YAW轴位置滤波器参数
    const static fp32 gimbal_pitch_pos_filter[1]   = {0.2f};   // PITCH轴位置滤波器参数


   // 初始化一阶低通滤波器
    first_order_filter_init(&pos_control->filter_rc_yaw_vel_set, 0.002f, gimbal_yaw_order_filter);     // 遥控YAW速度滤波器
    first_order_filter_init(&pos_control->filter_rc_pitch_vel_set, 0.002f, gimbal_pitch_order_filter); // 遥控PITCH速度滤波器

    // first_order_filter_init(&pos_control->filter_MiniPc_pitch_set, 0.002, gimbal_pitch_pos_filter);    // 小电脑PITCH位置滤波器
    // first_order_filter_init(&pos_control->filter_MiniPc_yaw_set, 0.002, gimbal_yaw_pos_filter);        // 小电脑YAW位置滤波器


    // 初始化PITCH轴PID控制器
    static const fp32 Pitch_speed_pid[3]    = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};                         // 速度环PID参数
    static const fp32 Pitch_absolute_pid[3] = {PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->pitch_motor_absolute_angle_pid, PID_POSITION, Pitch_absolute_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->pitch_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    // 初始化YAW轴PID控制器
    static const fp32 Yaw_speed_pid[3]    = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};                         // 速度环PID参数
    static const fp32 Yaw_absolute_pid[3] = {YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD}; // 角度环PID参数
    old_PID_Init(&pos_control->yaw_absolute_angle_pid, PID_POSITION, Yaw_absolute_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT);
    old_PID_Init(&pos_control->yaw_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);


    // 初始化目标位置
    pos_control->yaw_target_pos   = 0.0f;
    pos_control->pitch_target_pos = 0.0f;


    // 获取电机测量数据指针
    pos_control->pitch_motor_measure = get_pitch_motor_info();
    pos_control->yaw_motor_measure   = get_yaw_motor_info();


    // 初始化状态标志
    pos_control->calibrate_warning = false;

    // 设置初始电流为0，初始化相对位置
    pos_control->pitch_motor_measure->target_current = 0;
    pos_control->yaw_motor_measure->target_current = 0;
    // pos_control->yaw_relattive_pos                   = 0.0f;
    // pos_control->pitch_relatiive_pos                   = 0.0f;


    // 获取USB自瞄数据指针
    // pos_control->usb_autoAim_ptr = getUsbMiniPcPtr();
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
    gimbal->gimbal_RC   = get_remote_control_point(); // 遥控器数据

    gimbal->gimbal_mod  = GIMBAL_MOD_NO_FORCE;        // 初始模式为无力
    // gimbal->minipc_info = getUsbDpkgData();           // 小电脑通信数据
    // gimbal->ins_info    = get_ins_info_point();       // 惯性导航数据

    // 更新反馈数据并设置初始位置
    gimbal_feedback(gimbal);
    gimbal->gimbal_pos.pitch_target_pos = gimbal->gimbal_pos.pitch_motor_measure->realtime_position; // PITCH目标位置为当前角度
    gimbal->gimbal_pos.init_pitch_pos   = gimbal->gimbal_pos.pitch_motor_measure->realtime_position; // 记录PITCH初始位置

    gimbal->gimbal_pos.yaw_target_pos = gimbal->gimbal_pos.yaw_motor_measure->realtime_position; // YAW目标位置为当前角度
    gimbal->gimbal_pos.init_yaw_pos   = gimbal->gimbal_pos.yaw_motor_measure->realtime_position; // 记录YAW初始位置



    // 初始化其他状态变量
    gimbal->gimbal_pos.yaw_relattive_pos   = 0.0f;                      // YAW相对位置清零
    gimbal->gimbal_shoot.pull_target_speed = SHOOT_PULL_TOWARD_POSTIVE; // 拨弹轮初始目标速度
    gimbal->gimbal_shoot.pull_is_block     = false;                     // 拨弹轮无堵塞
    gimbal->gimbal_shoot.locker     = true;                             //   锁定状态
    gimbal->gimbal_pos.add_pitch           = 0.0f;                      // PITCH增量清零
    gimbal->gimbal_pos.add_yaw             = 0.0f;                      // YAW增量清零
}
///////////////////////////初始化结束//////////////////
/////////////////////////模式设置开始./////////////
/**
 * @brief 设置云台工作模式
 * @param gimbal_set_mod 云台控制结构体指针
 * @details 根据遥控器拨杆位置或串口命令设置云台工作模式，支持模式切换逻辑
 */
void gimbal_set_mod(gimbal_t *gimbal_set_mod)
{
    static TickType_t cal_time = 0; // 校准时间记录


    // 遥控器控制模式
    gimbal_set_mod->last_gimbal_mod = gimbal_set_mod->gimbal_mod;

    //////////////////////////////////////////////////////////////////////

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
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_motor_measure->realtime_position;
        gimbal_transfer->gimbal_pos.init_pitch_pos   = gimbal_transfer->gimbal_pos.pitch_motor_measure->realtime_position;

        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_motor_measure->realtime_position;
        gimbal_transfer->gimbal_pos.init_yaw_pos   = gimbal_transfer->gimbal_pos.yaw_motor_measure->realtime_position;

    }
    // 从无力模式切换到慢速校准模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_NO_FORCE && gimbal_transfer->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_motor_measure->realtime_position;
        gimbal_transfer->gimbal_pos.init_pitch_pos   = gimbal_transfer->gimbal_pos.pitch_motor_measure->realtime_position;

        gimbal_transfer->gimbal_pos.yaw_target_pos = gimbal_transfer->gimbal_pos.yaw_motor_measure->realtime_position;
        gimbal_transfer->gimbal_pos.init_yaw_pos   = gimbal_transfer->gimbal_pos.yaw_motor_measure->realtime_position;
    }
    // 从慢速校准模式切换到正常控制模式
    else if (gimbal_transfer->last_gimbal_mod == GIMBAL_MOD_SLOW_CALI && gimbal_transfer->gimbal_mod == GIMBAL_MOD_NORMAL) {
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;

    }
    // 切换到自动瞄准模式
    else if (gimbal_transfer->last_gimbal_mod != GIMBAL_MOD_AutoAim && gimbal_transfer->gimbal_mod == GIMBAL_MOD_AutoAim) {
        // 保留视觉目标设置的代码（已注释）
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_pitch = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        // gimbal_transfer->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
        gimbal_transfer->gimbal_pos.pitch_target_pos = gimbal_transfer->gimbal_pos.pitch_absolute_pos;
        gimbal_transfer->gimbal_pos.yaw_target_pos   = gimbal_transfer->gimbal_pos.yaw_absolute_pos;
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


    // 更新YAW轴相对位置（来自电机编码器）
    gimbal_feedback->gimbal_pos.yaw_relattive_pos += gimbal_feedback->gimbal_pos.yaw_motor_measure->realtime_position;
    // gimbal_feedback->gimbal_pos.yaw_relattive_pos = rad_format(gimbal_feedback->gimbal_pos.yaw_relattive_pos);

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
void gimbal_rc_set_control(gimbal_t *gimbal, float *add_yaw, float *add_pitch)
{
    if (gimbal == NULL) {
        return;
    }

    // 遥控器原始通道值
    int16_t wz_channel, pitch_channel;
    fp32 wz_set_channel, pitch_set_channel;

    // 死区限制，避免遥控器摇杆中位时的微小偏差
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_X_CH], wz_channel, 20);    // 右摇杆X轴控制YAW
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_Y_CH], pitch_channel, 20); // 右摇杆Y轴控制PITCH

    // 通道值转换为控制量（乘以灵敏度系数）
    wz_set_channel    = wz_channel * GIMBAL_WZ_RC_SEN;
    pitch_set_channel = pitch_channel * GIMBAL_PITCH_RC_SEN;


    // 一阶低通滤波，平滑控制输入
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_yaw_vel_set, wz_set_channel);
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_pitch_vel_set, pitch_set_channel);


    // 输出滤波后的控制增量
    *add_pitch = gimbal->gimbal_pos.filter_rc_pitch_vel_set.out;
    *add_yaw   = gimbal->gimbal_pos.filter_rc_yaw_vel_set.out;
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
    *add_pitch = PITCH_CALI_POS_1 -gimbal->gimbal_pos.pitch_target_pos / 800.0f;
    // 缓慢将ROLL轴移动到零位（除以500减慢速度，指数衰减）
    *add_yaw = YAW_CALI_POS_1 -gimbal->gimbal_pos.yaw_target_pos / 800.0f;

    // 当PITCH角度接近零位时，直接设置目标为零
    if (fabs(PITCH_CALI_POS_1 -gimbal->gimbal_pos.pitch_motor_measure->realtime_position) < 0.04f) {
        *add_pitch = PITCH_CALI_POS_1 - gimbal->gimbal_pos.pitch_target_pos;
    }
    // 当YAW角度接近零位时，直接设置目标为零
    if (fabs(YAW_CALI_POS_1 -gimbal->gimbal_pos.yaw_motor_measure->realtime_position) < 0.04f) {
        *add_yaw = YAW_CALI_POS_1 - gimbal->gimbal_pos.yaw_target_pos;
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
 * @brief 设置云台控制量
 * @param gimbal_set_control 云台控制结构体指针
 * @details 根据当前工作模式设置云台目标位置和发射轮目标转速
 */
void gimbal_set_control(gimbal_t *gimbal_set_control)
{
    float yaw_set = 0, pitch_set = 0, roll_set = 0;

    // 无力模式：停止发射，直接返回
    if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NO_FORCE) {
        gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
        return;
    }
    // 慢速校准模式：执行校准控制，停止发射
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_SLOW_CALI) {
        calibrate_control(gimbal_set_control, &yaw_set, &pitch_set);
        gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
    }
    // 正常控制模式：遥控器控制，停止发射
    else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_NORMAL) {

        gimbal_rc_set_control(gimbal_set_control, &yaw_set, &pitch_set);

        // gimbal_set_control->gimbal_shoot.shoot_target_speed = 0.0f;
    }

    // // 自动瞄准模式：视觉控制，启动发射轮
    // else if (gimbal_set_control->gimbal_mod == GIMBAL_MOD_AutoAim) {
    //     minipc_control(gimbal_set_control, &yaw_set, &pitch_set);
    //     gimbal_set_control->gimbal_shoot.shoot_target_speed = SHOOT_SPD;
    // }

    // 更新目标位置
    gimbal_set_control->gimbal_pos.last_pitch_target_pos = gimbal_set_control->gimbal_pos.pitch_target_pos;
    gimbal_set_control->gimbal_pos.pitch_target_pos += pitch_set;


    // 限制PITCH轴角度范围
    limitPos(&gimbal_set_control->gimbal_pos.pitch_target_pos, MAX_PITCH_RAD, MIN_PITCH_RAD);


    gimbal_set_control->gimbal_pos.last_yaw_target_pos = gimbal_set_control->gimbal_pos.yaw_target_pos;
    gimbal_set_control->gimbal_pos.yaw_target_pos += yaw_set;

    // 限制ROLL轴角度范围
    limitPos(&gimbal_set_control->gimbal_pos.yaw_target_pos, MAX_YAW_RAD, MIN_YAW_RAD);

    // // 处理跨零点旋转，选择最短路径
    // cross_roattion_handle(&gimbal_set_control->gimbal_pos.pitch_target_pos,
    //                       &gimbal_set_control->gimbal_pos.pitch_absolute_pos);
    //
    // // cross_roattion_handle(&gimbal_set_control->gimbal_pos.yaw_target_pos,
    // //                       &gimbal_set_control->gimbal_pos.yaw_absolute_pos);
    //
    // cross_roattion_handle(&gimbal_set_control->gimbal_pos.roll_target_pos,
    //                       &gimbal_set_control->gimbal_pos.roll_absolute_pos);
}
/**
 * @brief 检测拨弹轮是否卡弹
 * @param BlockDetect 发射系统结构体指针
 * @return 1-检测到卡弹, 0-卡弹解除, -2-状态无变化
 * @details 通过监测拨弹轮转速来判断是否发生卡弹，并提供防卡弹处理
 */
int isShootBlock(gimbal_shoot_t *BlockDetect)
{

    // 拨弹轮正常转动时（速度小于-0.5），更新无堵转时间
    if (BlockDetect->shoot_pull_rad < -0.5f) {
        BlockDetect->no_block_update_time = xTaskGetTickCount();
    }

    // 卡弹反转1秒后，重置无堵转时间，准备重新检测
    if (BlockDetect->pull_is_block == true && (xTaskGetTickCount() - BlockDetect->block_time > 1000)) {
        BlockDetect->no_block_update_time = xTaskGetTickCount();
        return 0; // 卡弹状态解除
    }

    // 持续1秒速度过小则判定为卡弹
    if (BlockDetect->pull_is_block == false && (xTaskGetTickCount() - BlockDetect->no_block_update_time > 1000)) {
        BlockDetect->block_time = xTaskGetTickCount();
        return 1; // 检测到卡弹
    }
    return -2; // 状态无变化
}
/**
 * @brief 计算云台发射系统控制量
 * @param shoot_control 发射系统结构体指针
 * @param shoot_flag 发射轮启动标志
 * @param fire_flag 开火标志
 * @details 控制发射轮转速、拨弹轮工作，包含防卡弹逻辑和发射准备检测
 */
void cal_gimbal_shoot_control(gimbal_shoot_t *shoot_control, bool_t shoot_flag, bool_t fire_flag)
{

    // 发射轮启动时计算PID控制量
    if (shoot_flag) {
        old_PID_Calc(&shoot_control->Shoot_left_PID, shoot_control->shoot_left_rad, shoot_control->shoot_target_speed);
        old_PID_Calc(&shoot_control->Shoot_right_PID, shoot_control->shoot_right_rad, shoot_control->shoot_target_speed);
        old_PID_Calc(&shoot_control->Pull_PID, shoot_control->shoot_pull_rad, shoot_control->pull_target_speed);
    }
    // 发射轮停止时清零PID并设置电流为0
    else {
        old_PID_clear(&shoot_control->Shoot_left_PID);
        old_PID_clear(&shoot_control->Shoot_right_PID);
        old_PID_clear(&shoot_control->Pull_PID);

        shoot_control->shoot_motor_right->target_current = 0;
        shoot_control->shoot_motor_left->target_current  = 0;
        shoot_control->pull_motor->target_current        = 0;
    }

    // 发射轮低速时使用锁定PID参数，提供更大的启动力矩
    if (shoot_control->shoot_left_rad < SHOOT_SPD / 2.0f && shoot_control->shoot_right_rad < SHOOT_SPD / 2.0f) {
        shoot_control->Shoot_right_PID.Kp = PID_SHOOT_LOCK_KP;
        shoot_control->Shoot_left_PID.Kp  = PID_SHOOT_LOCK_KP;
    }
    // 发射轮高速时使用正常PID参数
    else {
        shoot_control->Shoot_right_PID.Kp = PID_SHOOT_RIGHT_MOTOR_KP;
        shoot_control->Shoot_left_PID.Kp  = PID_SHOOT_LEFT_MOTOR_KP;
    }

    // 检查发射轮是否达到可发射速度（90%目标速度）
    if (shoot_control->shoot_left_rad > SHOOT_SPD * 0.9f && shoot_control->shoot_right_rad > SHOOT_SPD * 0.9f) {
        shoot_control->shoot_ready_flag = true; // 发射准备就绪
    } else
        shoot_control->shoot_ready_flag = false; // 发射未准备

    // 设置发射轮目标电流
    shoot_control->shoot_motor_right->target_current = (int16_t)shoot_control->Shoot_right_PID.out;
    shoot_control->shoot_motor_left->target_current  = (int16_t)shoot_control->Shoot_left_PID.out;

    // 当发射轮准备就绪且收到开火指令时，控制拨弹轮
    if (shoot_control->shoot_ready_flag == true && fire_flag == true) {
        shoot_control->pull_motor->target_current = (int16_t)shoot_control->Pull_PID.out;

        // 检测卡弹状态并更新拨弹轮堵塞标志
        int state = isShootBlock(shoot_control);
        if (state == 1) {
            shoot_control->pull_is_block = true; // 检测到卡弹
        } else if (state == 0) {
            shoot_control->pull_is_block = false; // 卡弹解除
        } else {
            // 状态无变化，保持当前状态
            shoot_control->pull_is_block = shoot_control->pull_is_block;
        }

        // 根据是否卡弹设置拨弹轮转向
        if (shoot_control->pull_is_block) {
            shoot_control->pull_target_speed = SHOOT_PULL_TOWARD_NEGTIVE; // 反转解卡
        } else
            shoot_control->pull_target_speed = SHOOT_PULL_TOWARD_POSTIVE; // 正转拨弹

    }
    // 未开火时拨弹轮电流为0
    else
        shoot_control->pull_motor->target_current = 0;
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
    gimbal_loop->gimbal_pos.pitch_motor_measure->target_speed =
        old_PID_Calc(&gimbal_loop->gimbal_pos.pitch_motor_absolute_angle_pid,
                              gimbal_loop->gimbal_pos.pitch_absolute_pos,
                              gimbal_loop->gimbal_pos.pitch_target_pos);


    // 注释掉的速度环PID控制代码
    // gimbal_loop->gimbal_pos.pitch_motor_measure->target_current = \
    //             (int16_t)old_PID_Calc(&gimbal_loop->gimbal_pos.gimbal_motor_gyro_pid,\
    //                                  gimbal_loop->ins_info->pit_gyro,\
    //                                 gimbal_loop->gimbal_pos.tmp);

    // PITCH轴重力补偿（基于当前角度的前馈控制）
    // gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor -= (int16_t)gimbal_loop->gimbal_pos.pitch_absolute_pos * 15000.0f;
    // gimbal_loop->gimbal_pos.pitch_motor_measure->target_tor += (int16_t)gimbal_loop->gimbal_pos.pitch_absolute_pos * 15000.0f;


    // YAW轴双环PID控制（角度环+速度环），排除无力和校准模式
    if (gimbal_loop->gimbal_mod != GIMBAL_MOD_NO_FORCE && gimbal_loop->gimbal_mod != GIMBAL_MOD_SLOW_CALI) {

        gimbal_loop->gimbal_pos.yaw_motor_measure->target_speed = old_PID_Calc(&gimbal_loop->gimbal_pos.yaw_absolute_angle_pid,
                                               gimbal_loop->gimbal_pos.yaw_absolute_pos,
                                               gimbal_loop->gimbal_pos.yaw_target_pos);

    }

    // 控制发射系统（发射轮常开，根据开火标志控制拨弹轮）
    // cal_gimbal_shoot_control(&gimbal_loop->gimbal_shoot,
    //                          true,
    //                          );


    // }

    // DBUS遥控器控制模式：右拨杆下拨时停止所有电机
    if (switch_is_down(gimbal_loop->gimbal_RC->rc.s[RIGHT_SWITCH]) ){
        gimbal_loop->gimbal_pos.pitch_motor_measure->target_speed = 0;
        gimbal_loop->gimbal_pos.yaw_motor_measure->target_speed    = 0;
        gimbal_loop->gimbal_shoot.shoot_motor_left->target_current    = 0;
        gimbal_loop->gimbal_shoot.shoot_motor_right->target_current   = 0;
        gimbal_loop->gimbal_shoot.pull_motor->target_current         = 0;
    }

}