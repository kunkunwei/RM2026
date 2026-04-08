#include "Chassis_task.h"
#include "main.h"
#include "remote_control.h"
#include "observe_task.h"
// ============================================================
#include "bsp_dwt.h"
#include "monitor.h"
#include "usart.h"
#include "vofa.h"

// ============================================================
/*统一将逆时针方向确认为YAW角度增长方向，使用右手系*/
/*
* 实际电机ID映射：
* 电机4: 左前轮 (LF) - 对应数组索引3
* 电机3: 右前轮 (RF) - 对应数组索引2
* 电机1: 左后轮 (LR) - 对应数组索引0
* 电机2: 右后轮 (RR) - 对应数组索引1
*
* 数组索引与电机位置对应关系：
* chassis_motor[0]: 左后轮 (LR) - 电机1
* chassis_motor[1]: 右后轮 (RR) - 电机2
* chassis_motor[2]: 右前轮 (RF) - 电机3
* chassis_motor[3]: 左前轮 (LF) - 电机4
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
// 限幅函数
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}
// 底盘运动数据
static chassis_move_t chassis_move;
// 从云台通信获取控制命令

// ============================================================

// ============================================================
// 底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
// 设置遥控器状态
void chassis_set_mode(chassis_move_t *chassis_move_mode);
// 切换模式的时候数据处理部分
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
// 底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
// 设置控制量
void chassis_set_contorl(chassis_move_t *chassis_move_control);
// loop
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
// ============================================================


void Chassis_Task(void const *argument)
{
    /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    // osDelay(100);
    /* 等待云台初始化完成后再注册回调，避免访问空指针 */
    while (!is_gimbal_init_done()) osDelay(1);
    TickType_t systick = 0;

    chassis_init(&chassis_move);
    for (;;)
    {
        systick = osKernelSysTick();
        chassis_set_mode(&chassis_move);
        // 遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        // 底盘数据更新
        chassis_feedback_update(&chassis_move);
        // //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        // 底盘控制PID计算
        chassis_control_loop(&chassis_move);

        // 心跳刷新
        // 系统监控 - 心跳和CPU记录（每2ms调用）
        SystemMonitor_Process();
        osDelayUntil(&systick, CHASSIS_CONTROL_TIME_MS);
    }
    /* USER CODE END Chassis_Task */
}

/**
 * @brief          底盘初始化
 * @author         pxx
 * @param          chassis_move_init   底盘结构体指针
 * @retval         void
 */
void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    uint8_t i;
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    // 一阶低通滤波初始化
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    // 底盘开机状态为无力、不接触地面
    chassis_move_init->mode.chassis_mode = CHASSIS_FORCE_RAW;
    chassis_move_init->mode.last_chassis_mode = CHASSIS_FORCE_RAW;

    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->VT03_ctrl = get_pc_uart_ctrl_point();
    // 获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_move_init->chassis_imu_gyro = get_gyro_data_point();
    chassis_move_init->chassis_imu_accel = get_accel_data_point();
    //获取YAW轴电机指针
    chassis_move_init->yaw_motor_measure = get_yaw_motor();
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {   // 获取底盘驱动轮电机指针
        chassis_move_init->chassis_motor[i].chassis_motor_measure = get_chassis_motor(i);
        old_PID_Init(&chassis_move_init->chassis_pid.motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化旋转PID
    old_PID_Init(&chassis_move_init->chassis_pid.chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    // 使用与X轴相同的滤波器参数初始化WZ滤波器
    first_order_filter_init(&chassis_move_init->state_set.chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    //小陀螺旋转 斜波函数缓启
    ramp_init(&chassis_move_init->rotation_ramp_wz, CHASSIS_CONTROL_TIME, ROTATION_SPEED_MAX, 0);

    //初始化超级模式通道的开关状态，防止开机时，遥控器未还原造成的意外 不可删去！！！！
    chassis_move_init->mode.last_fanction_channel = chassis_move_init->chassis_RC->rc.s[FUNCTION_CHANNEL];

    // ============================================================
    // 更新一下数据
    chassis_feedback_update(chassis_move_init);
}
/**
 * @brief          底盘数据更新
 * @author         pxx
 * @param          chassis_move_update 底盘结构体指针
 * @retval         void
 */
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度（将RPM转换为rad/s）
        // 参考代码中直接使用电机速度进行计算，这里我们使用标准转换
        // M3508电机：RPM转换为rad/s的系数是 2π/60 = 0.10471975512
        chassis_move_update->chassis_motor[i].speed = chassis_move_update->chassis_motor[i].chassis_motor_measure->rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
        chassis_move_update->chassis_motor[i].accel = chassis_move_update->chassis_pid.motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
    chassis_move_update->chassis_motor[0].speed = -chassis_move_update->chassis_motor[0].speed;
    chassis_move_update->chassis_motor[0].accel = -chassis_move_update->chassis_motor[0].accel;
    // chassis_move_update->chassis_motor[2].speed = -chassis_move_update->chassis_motor[2].speed;
    // chassis_move_update->chassis_motor[2].accel = -chassis_move_update->chassis_motor[2].accel;
    chassis_move_update->chassis_motor[1].speed = -chassis_move_update->chassis_motor[1].speed;
    chassis_move_update->chassis_motor[1].accel = -chassis_move_update->chassis_motor[1].accel;
    // chassis_move_update->chassis_motor[3].speed = -chassis_move_update->chassis_motor[3].speed;
    // chassis_move_update->chassis_motor[3].accel = -chassis_move_update->chassis_motor[3].accel;
    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    // chassis_move_update->state_ref.vx = (-chassis_move_update->chassis_motor[3].speed + chassis_move_update->chassis_motor[2].speed + chassis_move_update->chassis_motor[0].speed - chassis_move_update->chassis_motor[1].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    // chassis_move_update->state_ref.vy = (chassis_move_update->chassis_motor[3].speed + chassis_move_update->chassis_motor[2].speed - chassis_move_update->chassis_motor[0].speed - chassis_move_update->chassis_motor[1].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    // chassis_move_update->state_ref.wz=  (-chassis_move_update->chassis_motor[3].speed - chassis_move_update->chassis_motor[2].speed - chassis_move_update->chassis_motor[0].speed - chassis_move_update->chassis_motor[1].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;


    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->state_ref.vx = (-chassis_move_update->chassis_motor[0].speed + chassis_move_update->chassis_motor[1].speed + chassis_move_update->chassis_motor[2].speed - chassis_move_update->chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->state_ref.vy = (-chassis_move_update->chassis_motor[0].speed - chassis_move_update->chassis_motor[1].speed + chassis_move_update->chassis_motor[2].speed + chassis_move_update->chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->state_ref.wz = (-chassis_move_update->chassis_motor[0].speed - chassis_move_update->chassis_motor[1].speed - chassis_move_update->chassis_motor[2].speed - chassis_move_update->chassis_motor[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    // 更新云台相对底盘角度（云台正方向相对底盘正方向的角度）
    // 注意：云台电机角度逆时针为正，与底盘坐标系一致
    // gimbal_yaw_relative = 云台电机角度 - 云台电机零位
    // 云台电机零位是当云台正方向与底盘正方向对齐时的电机角度
    chassis_move_update->state_ref.gimbal_yaw_relative =rad_format( chassis_move_update->yaw_motor_measure->pos+2.87f);
    
    // 更新底盘相对云台角度（与云台相对底盘角度相反）
    // 底盘相对云台角度 = -云台相对底盘角度
    chassis_move_update->state_ref.chassis_yaw_relative = chassis_move_update->state_ref.gimbal_yaw_relative;
    
    // 更新底盘绝对角度（世界坐标系）
    // 注意：IMU角度顺时针为正，但底盘坐标系逆时针为正
    // 底盘绝对角度 = -IMU角度 - 底盘相对云台角度
    // 或者等价于：底盘绝对角度 = -IMU角度 + 云台相对底盘角度
    float imu_yaw = *(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET);
    chassis_move_update->state_ref.chassis_yaw_absolute = rad_format(-imu_yaw - chassis_move_update->state_ref.chassis_yaw_relative);

}
/**
 * @brief          设置底盘模式（支持云台控制和遥控器控制双模式）
 * @author         pxx
 * @param          chassis_move_mode   底盘结构体指针
 * @retval         void
 */
void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
      // pause按下切换：无力模式
    if (chassis_move_mode->VT03_ctrl->rc.pause == 1&&chassis_move_mode->VT03_pause_last==0 &&chassis_move_mode->mode.chassis_mode != CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_FORCE_RAW;
        chassis_move_mode->auto_aim_mode =false;
        chassis_move_mode->spinning_mode =false;
    }
    // 模式键0挡位且当前为无力模式：底盘跟随云台模式
    else if (chassis_move_mode->VT03_ctrl->rc.pause == 1&&chassis_move_mode->VT03_pause_last==0 &&chassis_move_mode->VT03_ctrl->rc.mode_sw==0&&chassis_move_mode->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode =CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW;

    }
    // 模式键1挡位且当前为无力模式：云台跟随底盘模式
    else if (chassis_move_mode->VT03_ctrl->rc.pause == 1&&chassis_move_mode->VT03_pause_last==0 &&chassis_move_mode->VT03_ctrl->rc.mode_sw==1&&chassis_move_mode->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode =  CHASSIS_GIMBAL_FOLLOW_CHASSIS ;

    }
    // 模式键2挡位且当前为无力模式：底盘向量控制但不跟随云台模式
    else if (chassis_move_mode->VT03_ctrl->rc.pause == 1&&chassis_move_mode->VT03_pause_last==0 &&chassis_move_mode->VT03_ctrl->rc.mode_sw==2&&chassis_move_mode->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;

    }
    // 模式键0挡位且当前为其它非无力模式：进入底盘跟随云台模式
    else if (chassis_move_mode->VT03_ctrl->rc.mode_sw==0&&chassis_move_mode->mode.chassis_mode != CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW &&chassis_move_mode->mode.chassis_mode !=CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    // 模式键1挡位且当前为其它非无力模式：进入云台跟随底盘模式
    else if (chassis_move_mode->VT03_ctrl->rc.mode_sw==1&&chassis_move_mode->mode.chassis_mode != CHASSIS_GIMBAL_FOLLOW_CHASSIS &&chassis_move_mode->mode.chassis_mode !=CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode =  CHASSIS_GIMBAL_FOLLOW_CHASSIS;
    }
    // 模式键2挡位且当前为其它非无力模式：进入底盘向量控制但不跟随云台模式。此时云台可进入自瞄模式，底盘遥控器前进方向为机器人正方向
    else if (chassis_move_mode->VT03_ctrl->rc.mode_sw==2&&chassis_move_mode->mode.chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW &&chassis_move_mode->mode.chassis_mode !=CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->mode.chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    /*--------------------------------------------------自瞄和小陀螺功能切换-------------------------------------------------------*/
    // // 右fn_2按下为1且当前已开启自瞄：退出自瞄
    // if (chassis_move_mode->VT03_ctrl->rc.fn_2 == 1 && (chassis_move_mode->VT03_fn2_last == false) && chassis_move_mode->auto_aim_mode == true)
    // {
    //     chassis_move_mode->auto_aim_mode =false;
    // }
    // // 右fn_2按下为1：切换并进入自动瞄准模式
    // else if (chassis_move_mode->VT03_ctrl->rc.fn_2 == 1 && (chassis_move_mode->VT03_fn2_last == false) && chassis_move_mode->auto_aim_mode == false && chassis_move_mode->mode.chassis_mode !=CHASSIS_FORCE_RAW)
    // {
    //     chassis_move_mode->auto_aim_mode =true;
    // }

    ///// 小陀螺切换：左fn_1按下为1 且处于正常模式时激活 //////////
    if (chassis_move_mode->VT03_ctrl->rc.fn_1== 1 && (chassis_move_mode->VT03_fn1_last == false) && chassis_move_mode->spinning_mode == false && chassis_move_mode->mode.chassis_mode !=CHASSIS_FORCE_RAW)
    {
        chassis_move_mode->spinning_mode = true;
    }
    else if  (chassis_move_mode->VT03_ctrl->rc.fn_1== 1 && (chassis_move_mode->VT03_fn1_last == false) && chassis_move_mode->spinning_mode == true )
    {
        chassis_move_mode->spinning_mode = false;
    }

    // 更新上次遥控器模式状态
   chassis_move_mode->VT03_fn1_last   = chassis_move_mode->VT03_ctrl->rc.fn_1;
   chassis_move_mode->VT03_fn2_last   = chassis_move_mode->VT03_ctrl->rc.fn_2;
   chassis_move_mode->VT03_pause_last = chassis_move_mode->VT03_ctrl->rc.pause;
   chassis_move_mode->mode.last_chassis_mode = chassis_move_mode->mode.chassis_mode;

}
/**
 * @brief          遥控器状态切换数据保存
 * @author         pxx
 * @param          chassis_move_transit    底盘结构体指针
 * @retval         void
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->mode.last_chassis_mode == chassis_move_transit->mode.chassis_mode)
    {
        return;
    }

    // 切入跟随云台模式（云台正方向为机器人正方向）
    if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW) &&
         chassis_move_transit->mode.chassis_mode      == CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        // 底盘按照转动最小的角度转到云台YAW角度为0，做到双方的正方向重合
        // 计算最小转动角度：将底盘当前yaw角度调整到0度（云台正方向）
        // chassis_move_transit->chassis_yaw_set = 0.0f;
        // chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        chassis_move_transit->state_set.chassis_yaw_set = chassis_move_transit->state_ref.chassis_yaw_absolute ;

    }
    // 切入云台跟随底盘模式（底盘正方向为机器人正方向）
    else if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_GIMBAL_FOLLOW_CHASSIS) &&
              chassis_move_transit->mode.chassis_mode      == CHASSIS_GIMBAL_FOLLOW_CHASSIS)
    {
        // 云台需要按照转动最小的角度转到底盘当前YAW角度，做到双方的正方向重合
        // 底盘保持当前朝向，云台会跟随底盘
        chassis_move_transit->state_set.chassis_yaw_set =chassis_move_transit->state_ref.chassis_yaw_absolute;
    }
    //切入不跟随云台模式（底盘正方向为机器人正方向）
    else if (chassis_move_transit->mode.last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW &&
              chassis_move_transit->mode.chassis_mode      == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->state_set.chassis_yaw_set =chassis_move_transit->state_ref.chassis_yaw_absolute;
    }

    // 切入小陀螺模式（云台正方向为机器人正方向）
    if ((chassis_move_transit->last_spinning_mode != true) &&
         chassis_move_transit->spinning_mode == true)
    {
        // 小陀螺模式：云台正方向作为机器人正方向，底盘会持续旋转
        // 保持云台相对世界坐标系下的角度不变（IMU角度）
        chassis_move_transit->state_set.chassis_yaw_set = chassis_move_transit->state_ref.chassis_yaw_absolute;
    }
    // 退出小陀螺处理
    if (chassis_move_transit->last_spinning_mode == true &&
        chassis_move_transit->spinning_mode == false) {
        // 计算最小转动角度，使底盘正方向与云台正方向重合
        float target_yaw = chassis_move_transit->state_ref.chassis_yaw_absolute;
        float current_yaw = chassis_move_transit->state_ref.chassis_yaw_absolute;
        float delta_yaw = target_yaw - current_yaw;

        // 选择最短路径
        if (delta_yaw > PI) delta_yaw -= 2*PI;
        if (delta_yaw < -PI) delta_yaw += 2*PI;

        chassis_move_transit->state_set.chassis_yaw_set = current_yaw + delta_yaw;
        }

    // 切入无力模式，清空里程计
    if ((chassis_move_transit->mode.last_chassis_mode != CHASSIS_FORCE_RAW) &&
              chassis_move_transit->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {

        chassis_move_transit->auto_aim_mode = false;
        chassis_move_transit->spinning_mode = false;
    }

    // chassis_move_transit->mode.last_chassis_mode = chassis_move_transit->mode.chassis_mode;
}
void keyboard_control(fp32 *vx_set, fp32 *vy_set,chassis_move_t *chassis_keyboard)
{
    float vx_channel,vy_channel ;
    // 前后控制（W/S）
    if(PC_KeyBoard_W && !PC_KeyBoard_S)
    {
        // 前进
        if(PC_KeyBoard_SHIFT)
            vx_channel = 600;      // 快速前进
        else if(PC_KeyBoard_CTRL)
            vx_channel = 150;      // 慢速前进
        else
            vx_channel = 300;      // 正常前进
    }
    else if(PC_KeyBoard_S && !PC_KeyBoard_W)
    {
        // 后退
        if(PC_KeyBoard_SHIFT)
            vx_channel = -600;     // 快速后退
        else if(PC_KeyBoard_CTRL)
            vx_channel = -150;     // 慢速后退
        else
            vx_channel = -300;     // 正常后退
    }

    // vy平移控制（A/D）
    if(PC_KeyBoard_A && !PC_KeyBoard_D)
    {
        // 左移
        if(PC_KeyBoard_SHIFT)
            vy_channel = 600;    // 快速逆时针
        else if(PC_KeyBoard_CTRL)
            vy_channel = 150;    // 慢速逆时针
        else
            vy_channel = 300;    // 正常逆时针
    }
    else if(PC_KeyBoard_D && !PC_KeyBoard_A)
    {
        // 右移
        if(PC_KeyBoard_SHIFT)
            vy_channel = -600;   // 快速顺时针
        else if(PC_KeyBoard_CTRL)
            vy_channel = -150;   // 慢速顺时针
        else
            vy_channel = -300;   // 正常顺时针
    }
    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_keyboard->state_set.chassis_cmd_slow_set_vx, vx_channel);
    first_order_filter_cali(&chassis_keyboard->state_set.chassis_cmd_slow_set_vy, vy_channel);
    *vx_set=chassis_keyboard->state_set.chassis_cmd_slow_set_vx.out;
    *vy_set=chassis_keyboard->state_set.chassis_cmd_slow_set_vy.out;
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
 * @brief          遥控器的数据处理成底盘的前进vx速度，vy速度
 * @author         pxx
 * @param          vx_set  x轴前进速度设置，m/s
 * @param          vy_set  y轴前进速度设置，m/s
 * @param          chassis_move_rc_to_vector   底盘结构体指针
 * @retval         void
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set,fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || angle_set == NULL)
    {
        return;
    }
    // 遥控器原始通道值
    int16_t vx_channel = 0, vy_channel = 0, angle_channel = 0;
    fp32 vx_set_channel = 0.0f, vy_set_channel = 0.0f, angle_set_channel = 0.0f;

    // 总是处理遥控器输入，即使摇杆在中位
    // if (chassis_move_rc_to_vector->mode.chassis_mode == CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW)
    // {
        // rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_Y_CH], vx_channel, CHASSIS_RC_DEADLINE); // 左摇杆Y轴控制vx
        // rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_X_CH], vy_channel, CHASSIS_RC_DEADLINE);    // 左摇杆X轴控制vy

    // }
    // else if (chassis_move_rc_to_vector->mode.chassis_mode == CHASSIS_GIMBAL_FOLLOW_CHASSIS)
    // {
        // rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_Y_CH], vx_channel, CHASSIS_RC_DEADLINE); // 左摇杆Y轴控制vx
        // rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_X_CH], vy_channel, CHASSIS_RC_DEADLINE);    // 左摇杆X轴控制vy
       // rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_RIGHT_X_CH], angle_channel, CHASSIS_RC_DEADLINE);    // 右摇杆X轴控制wz
    // }
    // else
    // {
        rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_Y_CH], vx_channel, CHASSIS_RC_DEADLINE); // 左摇杆Y轴控制vx
        rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_LEFT_X_CH], vy_channel, CHASSIS_RC_DEADLINE);    // 左摇杆X轴控制vy
        rc_deadline_limit(chassis_move_rc_to_vector->VT03_ctrl->rc.ch[RC_RIGHT_X_CH], angle_channel, CHASSIS_RC_DEADLINE);    // 右摇杆X轴控制wz
    // }

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = -vy_channel * CHASSIS_VY_RC_SEN;
    angle_set_channel = angle_channel * CHASSIS_ANGLE_Z_RC_SEN;

    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vy, vy_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz, angle_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vy.out = 0.0f;
    }
    // if (angle_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_Z_RC_SEN && angle_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_Z_RC_SEN)
    // {
    //     chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz.out = 0.0f;
    // }

    *vx_set = chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_vy.out;
    // *angle_set = 0;
    *angle_set = chassis_move_rc_to_vector->state_set.chassis_cmd_slow_set_wz.out;
    // 键盘控制，优先级高于遥控器输入
    // keyboard_control(vx_set,vy_set,chassis_move_rc_to_vector);

}
/**
  * @brief          麦轮运动分解（逆解）
  * @author         pxx
  * @param          vx_set  x轴前进速度设置，m/s
  * @param          vy_set  y轴前进速度设置，m/s
  * @param          wz_set  z轴旋转速度设置，rad/s
  * @param          wheel_speed 四个轮子的转速
  * @retval         void
  */
void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 2 ，3 旋转的速度变慢， 后面两轮 0,1 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) *  MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) *MOTOR_DISTANCE_TO_CENTER * wz_set;
}


/**
 * @brief          设置遥控器输入控制量
 * @author         pxx
 * @param          chassis_move_control    底盘结构体指针
 * @retval         void
 */
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    // 设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    uint8_t i=0;
    //获取遥控器输入的控制量
    chassis_rc_to_control_vector(&vx_set, &vy_set, &angle_set,chassis_move_control);
    //无力模式
    if (chassis_move_control->mode.chassis_mode == CHASSIS_FORCE_RAW)
    {
        vx_set = 0.0f;
        vy_set = 0.0f;
        angle_set = 0.0f;
        return;
    }
    //底盘跟随云台模式（云台正方向为机器人正方向）
     if (chassis_move_control->mode.chassis_mode == CHASSIS_CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        /*              v_x
       *                ^
       *                |
       *                |
       *                |
       * v_y<-----------|       云台速度映射
       */

         fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
         //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
         sin_yaw = arm_sin_f32(chassis_move_control->state_ref.gimbal_yaw_relative);
         cos_yaw = arm_cos_f32(chassis_move_control->state_ref.gimbal_yaw_relative);
         chassis_move_control->state_set.vx = cos_yaw * vx_set - sin_yaw * vy_set;
         chassis_move_control->state_set.vy = sin_yaw * vx_set + cos_yaw * vy_set;

         // 添加云台角速度前馈补偿： 底盘需要提高旋转响应速度
         // 底盘逆时针转(正角速度) → 云台逆时针转(负角速度，还没从IMU那里改，需要统一方向)
         // float feedforward_gain = 1.0f; // 前馈增益，可调节
         // yaw_motor_gyro_set -= chassis_yaw_rate * feedforward_gain;

         // 限制最大速度设定值
         // const float MAX_GYRO_SET = 10.0f; // rad/s
         // if (yaw_motor_gyro_set > MAX_GYRO_SET) yaw_motor_gyro_set = MAX_GYRO_SET;
         // if (yaw_motor_gyro_set < -MAX_GYRO_SET) yaw_motor_gyro_set = -MAX_GYRO_SET;
         //设置控制相对云台角度
         chassis_move_control->state_set.chassis_yaw_set = rad_format(0-chassis_move_control->state_ref.gimbal_yaw_relative);
         // chassis_move_control->state_set.chassis_yaw_set = rad_format(angle_set);
         //计算旋转PID角速度
         chassis_move_control->state_set.wz = old_PID_Calc(&chassis_move_control->chassis_pid.chassis_angle_pid, chassis_move_control->state_ref.gimbal_yaw_relative, chassis_move_control->state_set.chassis_yaw_set);
         //速度限幅
         chassis_move_control->state_set.wz = fp32_constrain(chassis_move_control->state_set.wz, -NORMAL_MAX_CHASSIS_SPEED_WZ, NORMAL_MAX_CHASSIS_SPEED_WZ);
         chassis_move_control->state_set.vx = fp32_constrain(chassis_move_control->state_set.vx, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
         chassis_move_control->state_set.vy = fp32_constrain(chassis_move_control->state_set.vy, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
         // 在小陀螺模式下添加科里奥利力补偿
         if (chassis_move_control->spinning_mode) {
             // 获取底盘旋转角速度
             float omega_z = chassis_move_control->state_ref.wz;

             // 科里奥利力补偿系数（需要根据实际系统调整）
             float coriolis_gain = 0.1f; // 经验值，需要实验调整

             // 计算科里奥利加速度补偿
             float coriolis_accel_x = 2.0f * omega_z * chassis_move_control->state_set.vy * coriolis_gain;
             float coriolis_accel_y = -2.0f * omega_z * chassis_move_control->state_set.vx * coriolis_gain;

             // 将加速度补偿转换为速度补偿（积分形式）
             static float coriolis_vel_x = 0.0f, coriolis_vel_y = 0.0f;
             coriolis_vel_x += coriolis_accel_x * CHASSIS_CONTROL_TIME;
             coriolis_vel_y += coriolis_accel_y * CHASSIS_CONTROL_TIME;

             // 应用补偿
             chassis_move_control->state_set.vx += coriolis_vel_x;
             chassis_move_control->state_set.vy += coriolis_vel_y;

             // 限幅
             chassis_move_control->state_set.vx = fp32_constrain(
                 chassis_move_control->state_set.vx,
                 -NORMAL_MAX_CHASSIS_SPEED_X,
                 NORMAL_MAX_CHASSIS_SPEED_X
             );
             chassis_move_control->state_set.vy = fp32_constrain(
                 chassis_move_control->state_set.vy,
                 -NORMAL_MAX_CHASSIS_SPEED_Y,
                 NORMAL_MAX_CHASSIS_SPEED_Y
             );
         }


    }// 云台跟随底盘模式（底盘正方向为机器人正方向）
    else if (chassis_move_control->mode.chassis_mode == CHASSIS_GIMBAL_FOLLOW_CHASSIS)
    {
    //底盘速度不需要旋转控制，直接映射遥控器输入
    // chassis_move_control->state_set.wz = angle_set;
    
    // ============================================================
    // 添加加速度限制前馈控制（与C++项目相同）
    // ============================================================
    // 计算速度误差
    float dvx = vx_set - chassis_move_control->state_ref.vx;
    float dvy = vy_set - chassis_move_control->state_ref.vy;
    float dwz = angle_set - chassis_move_control->state_ref.wz;
    
    // 计算速度误差的平方和
    float vsq = dvx * dvx + dvy * dvy;
    
    // 加速度限制（与C++项目相同的参数）
    static const float CHASSIS_MAX_ACCEL = 800000.0f;  // 与C++项目相同
    static const float CHASSIS_MAX_ALPHA = 800000.0f;  // 与C++项目相同
    static const float T_SAMPLE = 0.002f;              // 2ms控制周期
    
    // 计算加速度限制系数
    float k_ref = CHASSIS_MAX_ACCEL * T_SAMPLE / sqrtf(vsq + 0.0001f);
    float k_omega_ref = CHASSIS_MAX_ALPHA * T_SAMPLE / (dwz > 0 ? dwz : -dwz + 0.0001f);
    
    // 应用加速度限制
    if (k_ref < 1.0f) {
        dvx *= k_ref;
        dvy *= k_ref;
    }
    if (k_omega_ref < 1.0f) {
        dwz *= k_omega_ref;
    }
    
    // 计算加速度限制后的目标速度
    float vx_accel_limit = chassis_move_control->state_ref.vx + dvx;
    float vy_accel_limit = chassis_move_control->state_ref.vy + dvy;
    float wz_accel_limit = chassis_move_control->state_ref.wz + dwz;
    
    //速度限幅
    chassis_move_control->state_set.wz = fp32_constrain(wz_accel_limit, -NORMAL_MAX_CHASSIS_SPEED_WZ, NORMAL_MAX_CHASSIS_SPEED_WZ);
    chassis_move_control->state_set.vx = fp32_constrain(vx_accel_limit, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
    chassis_move_control->state_set.vy = fp32_constrain(vy_accel_limit, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
    
    // 调试输出
    static uint32_t debug_counter = 0;
    if (debug_counter++ % 100 == 0) {
        uart_printf(&huart6, "wz_in:%.2f wz_out:%.2f k_omega:%.3f\r\n",
            angle_set, chassis_move_control->state_set.wz, k_omega_ref);
    }
    }
    else if (chassis_move_control->mode.chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(chassis_move_control->state_ref.gimbal_yaw_relative);
        cos_yaw = arm_cos_f32(chassis_move_control->state_ref.gimbal_yaw_relative);
        chassis_move_control->state_set.vx = cos_yaw * vx_set - sin_yaw * vy_set;
        chassis_move_control->state_set.vy = sin_yaw * vx_set + cos_yaw * vy_set;
        //设置控制相对云台角度
        // chassis_move_control->state_set.chassis_yaw_set = rad_format(angle_set);
        //速度限幅
        chassis_move_control->state_set.wz = fp32_constrain(angle_set, -NORMAL_MAX_CHASSIS_SPEED_WZ, NORMAL_MAX_CHASSIS_SPEED_WZ);
        chassis_move_control->state_set.vx = fp32_constrain(chassis_move_control->state_set.vx, -NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X);
        chassis_move_control->state_set.vy = fp32_constrain(chassis_move_control->state_set.vy, -NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y);
    }

}

/**
 * @brief          底盘控制PID计算
 * @author         pxx
 * @param          chassis_move_control_loop   底盘结构体指针
 * @retval         void
 */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if (chassis_move_control_loop->mode.chassis_mode == CHASSIS_FORCE_RAW )
    {
        chassis_move_control_loop->chassis_motor[0].target_current = 0;
        chassis_move_control_loop->chassis_motor[1].target_current = 0;
        chassis_move_control_loop->chassis_motor[2].target_current = 0;
        chassis_move_control_loop->chassis_motor[3].target_current = 0;

        old_PID_clear(&chassis_move_control_loop->chassis_pid.motor_speed_pid[0]);
        old_PID_clear(&chassis_move_control_loop->chassis_pid.motor_speed_pid[1]);
        old_PID_clear(&chassis_move_control_loop->chassis_pid.motor_speed_pid[2]);
        old_PID_clear(&chassis_move_control_loop->chassis_pid.motor_speed_pid[3]);
        old_PID_clear(&chassis_move_control_loop->chassis_pid.chassis_angle_pid);
        return;
    }


    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->state_set.vx,
                                          chassis_move_control_loop->state_set.vy,
                                          chassis_move_control_loop->state_set.wz, wheel_speed);
    // uart_printf(&huart6, "wz:%.2f,speed : %.2f\r\n",chassis_move_control_loop->state_set.wz,wheel_speed[0]);
    //计算pid
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i].speed_set = (int16_t)wheel_speed[i];

    }
    for (i = 0; i < 4; i++)
    {
        if (chassis_move_control_loop->chassis_motor[i].speed_set>MAX_WHEEL_SPEED)
        {
            chassis_move_control_loop->chassis_motor[i].speed_set=MAX_WHEEL_SPEED;
        }
        else if (chassis_move_control_loop->chassis_motor[i].speed_set<-MAX_WHEEL_SPEED)
        {
            chassis_move_control_loop->chassis_motor[i].speed_set=-MAX_WHEEL_SPEED;
        }
    }
    for (i = 0; i < 4; i++)
    {
        old_PID_Calc(&chassis_move_control_loop->chassis_pid.motor_speed_pid[i], chassis_move_control_loop->chassis_motor[i].speed, chassis_move_control_loop->chassis_motor[i].speed_set);
        //赋值电流值
    }
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i].target_current = (int16_t)(chassis_move_control_loop->chassis_pid.motor_speed_pid[i].out);

    }


    // // 调试输出：检查控制量是否正常
    // static uint32_t debug_counter = 0;
    // if (debug_counter++ % 100 == 0) { // 每200ms输出一次（2ms周期 * 100）
    //     uart_printf(&huart6,"vx:%.2f vy:%.2f wz:%.2f ws0:%.2f cur:%d spd:%.2f\r\n",
    //         chassis_move_control_loop->state_set.vx,
    //         chassis_move_control_loop->state_set.vy,
    //         chassis_move_control_loop->state_set.wz,
    //         wheel_speed[0],
    //         chassis_move_control_loop->chassis_motor[0].target_current,
    //         chassis_move_control_loop->chassis_motor[0].speed);
    // }
}


// 获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void)
{
    return &chassis_move;
}
const Chassis_ref_t *get_chassis_ref_point(void)
{
    return &chassis_move.state_ref;
}