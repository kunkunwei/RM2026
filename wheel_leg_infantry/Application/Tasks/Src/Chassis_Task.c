#include "Chassis_task.h"
#include "main.h"
#include "remote_control.h"
#include "observe_task.h"
// ============================================================
// 新增算法模块头文件
#include "bsp_dwt.h"

#include "leg_angular_predictor.h"

#include "monitor.h"
#include "slip_detector.h"

#include "mpc_controller.h"
#include "usart.h"
#include "vofa.h"

// ============================================================

#define rc_deadline_limit(input, output, dealine)          \
    {                                                      \
        if ((input) > (dealine) || (input) < -(dealine)) { \
            (output) = (input);                            \
        } else {                                           \
            (output) = 0;                                  \
        }                                                  \
    }
//底盘运动数据
static chassis_move_t chassis_move;

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


// ===========================================================
static bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);

void Chassis_Task(void const *argument)
{
    /* USER CODE BEGIN Chassis_Task */
    /* Infinite loop */
    osDelay(100);
    TickType_t systick = 0;
    chassis_init(&chassis_move);
    // float time=0.0f;
    for (;;) {
        systick = osKernelSysTick();
        // time=DWT_GetTimeline_s();
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",chassis_move.right_leg.leg_angle)
        chassis_set_mode(&chassis_move);
        // 遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        // 底盘数据更新
        chassis_feedback_update(&chassis_move);
        // //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        // 底盘控制PID计算
        chassis_control_loop(&chassis_move);
        // jump_test(&chassis_move);
        //心跳刷新
        // 系统监控 - 心跳和CPU记录（每2ms调用）
        SystemMonitor_Process();
        // printf("%.2f,%.2f\r\n",chassis_move.right_leg.leg_length,chassis_move.left_leg.leg_length);
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
    if (chassis_move_init == NULL) {
        return;
    }
    chassis_move_init->left_leg.leg_length_set  = LEG_LENGTH_INIT;
    chassis_move_init->right_leg.leg_length_set = LEG_LENGTH_INIT;

    chassis_move_init->right_leg.touching_ground = true;
    chassis_move_init->left_leg.touching_ground  = true;
    chassis_move_init->now_tick                  = xTaskGetTickCount();
    chassis_move_init->last_out_ground_tick      = xTaskGetTickCount();

    chassis_move_init->position_control_intervention = false;
    // chassis_move_init->position_control_intervention = true;

    chassis_move_init->mit_normal_kd = MIT_KD_NORMAL;

    chassis_move_init->jump_state.jump_flag  = false;
    chassis_move_init->jump_state.last_jump_flag  = false;
    chassis_move_init->jump_state.jump_stage = 0;
    chassis_move_init->jump_state.current_time = 0.0f;
    chassis_move_init->jump_state.takeoff_start_time = 0.0f;
    chassis_move_init->jump_state.last_jump_finish_time = 0.0f;

    //力矩置零
    chassis_move_init->jump_state.jump_comtorque[0]=0.0f;
    chassis_move_init->jump_state.jump_comtorque[1]=0.0f;
    chassis_move_init->jump_state.jump_comtorque[2]=0.0f;
    chassis_move_init->jump_state.jump_comtorque[3]=0.0f;

    chassis_move_init->jump_state.takeoff_time = 0.0f;
    chassis_move_init->jump_state.landing_time = 0.0f;
    chassis_move_init->jump_state.takeoff_leg_length = 0.0f;

    chassis_move_init->jump_state.landing_velocity_x = 0.0f;
    chassis_move_init->leg_length_in_sky     = LEG_LENGTH_BUFFER;

    chassis_move_init->chassis_move_toward      = CHASSIS_MOVE_AHEAD;
    chassis_move_init->chassis_move_last_toward = CHASSIS_MOVE_AHEAD;
    chassis_move_init->change_toward_flag       = false;

    chassis_move_init->kilometer = 0.0f;
    // 轮腿底盘pid值
    const static fp32 leg_length_pid[3] = {LEG_LENGTH_PID_KP, LEG_LENGTH_PID_KI, LEG_LENGTH_PID_KD};

    const static fp32 leg_angle_err_pid[3] = {ANGLE_ERR_PID_KP, ANGLE_ERR_PID_KI, ANGLE_ERR_PID_KD};
    const static fp32 leg_angle_dot_pid[3] = {ANGLE_DOT_PID_KP, ANGLE_DOT_PID_KI, ANGLE_DOT_PID_KD};

    const static fp32 roll_l_pid[3] = {ROLL_CTRL_L_PID_KP, ROLL_CTRL_L_PID_KI, ROLL_CTRL_L_PID_KD};
    const static fp32 roll_f_pid[3] = {ROLL_CTRL_F_PID_KP, ROLL_CTRL_F_PID_KI, ROLL_CTRL_F_PID_KD};
    // 底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3]      = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 chassis_yaw_gyro_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    // ROS_PID
    const static fp32 chassis_ros_pid[3] = {ROS_WZ_PID_KP, ROS_WZ_PID_KI, ROS_WZ_PID_KD};

    // 一阶低通滤波初始化
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 state_xdot_constant[1]    = {0};
    const static fp32 chassis_x_brake[1]        = {0.05};

    // 底盘开机状态为无力、不接触地面
    chassis_move_init->chassis_mode   = CHASSIS_FORCE_RAW;
    chassis_move_init->touchingGroung = false;
    // 获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->chassis_can_rc_info = get_Chassis_CAN_RC_Info();
    // 获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    chassis_move_init->chassis_imu_gyro  = get_gyro_data_point();
    chassis_move_init->chassis_imu_accel = get_accel_data_point();
    // 获取关节电机指针
    chassis_move_init->right_leg.front_joint.joint_motor_measure = get_Joint_Motor_Measure_Point(3);
    chassis_move_init->right_leg.back_joint.joint_motor_measure  = get_Joint_Motor_Measure_Point(2);
    chassis_move_init->left_leg.back_joint.joint_motor_measure   = get_Joint_Motor_Measure_Point(1);
    chassis_move_init->left_leg.front_joint.joint_motor_measure  = get_Joint_Motor_Measure_Point(0);
    // 获取驱动轮电机指针
    chassis_move_init->right_leg.wheel_motor.wheel_motor_measure = get_Right_Wheel_Motor_Measure_Point();
    chassis_move_init->left_leg.wheel_motor.wheel_motor_measure  = get_Left_Wheel_Motor_Measure_Point();

    //获取打滑检测器指针
    chassis_move_init->slip_detector = get_slip_detector_point();
    //获取跳跃控制器指针
    // chassis_move_init->jump_ctl_ptr = &jump_ctrl;
    // 初始化PID 运动
    old_PID_Init(&chassis_move_init->left_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);
    old_PID_Init(&chassis_move_init->right_leg_length_pid, PID_POSITION, leg_length_pid, LEG_LENGTH_PID_MAX_OUT, LEG_LENGTH_PID_MAX_IOUT);

    // 初始双腿误差控制pid
    old_PID_Init(&chassis_move_init->angle_err_pid, PID_POSITION, leg_angle_err_pid, ANGLE_ERR_PID_MAX_OUT, ANGLE_ERR_PID_MAX_IOUT);
    old_PID_Init(&chassis_move_init->angle_dot_pid, PID_POSITION, leg_angle_dot_pid, ANGLE_DOT_PID_MAX_OUT, ANGLE_DOT_PID_MAX_IOUT);
    // 初始化横滚角pid
    old_PID_Init(&chassis_move_init->roll_ctrl_l_pid, PID_POSITION, roll_l_pid, ROLL_CTRL_L_PID_MAX_OUT, ROLL_CTRL_L_PID_MAX_IOUT);
    old_PID_Init(&chassis_move_init->roll_ctrl_f_pid, PID_POSITION, roll_f_pid, ROLL_CTRL_F_PID_MAX_OUT, ROLL_CTRL_F_PID_MAX_IOUT);
    // 初始化旋转PID
    old_PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    old_PID_Init(&chassis_move_init->chassis_yaw_gyro_pid, PID_POSITION, chassis_yaw_gyro_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    // ros
    old_PID_Init(&chassis_move_init->chassis_ros_wz_pid, PID_POSITION, chassis_ros_pid, ROS_WZ_PID_MAX_OUT, ROS_WZ_PID_MAX_IOUT);
        // 用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->state_xdot_filter, CHASSIS_CONTROL_TIME, state_xdot_constant);
    first_order_filter_init(&chassis_move_init->chassis_cmd_brake_vx, CHASSIS_CONTROL_TIME, chassis_x_brake);
    // 关节电机机械零点设置
    //******************************************???//
    //  chassis_move_init->left_leg.front_joint.offset_ecd = 7632;
    //  chassis_move_init->left_leg.back_joint.offset_ecd = 5932;
    //  chassis_move_init->right_leg.front_joint.offset_ecd = 6234;
    //  chassis_move_init->right_leg.back_joint.offset_ecd = 7307;

    // 关节电机限制角度，实际上是限制腿长
    chassis_move_init->leg_length_set = LEG_LENGTH_INIT;
    chassis_move_init->leg_length_max = LEG_LENGTH_MAX;
    chassis_move_init->leg_length_min = LEG_LENGTH_MIN;
    // 最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
    // 初始是倒地的
    chassis_move_init->is_conversely = true;


    // ============================================================
    // 初始化新增算法模块


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
    if (chassis_move_update == NULL) {
        return;
    }
    // extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t *Usb_receive_data;
    // 计算底盘姿态角度，陀螺仪需要在底盘上
    // 陀螺仪数据映射
    chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = -rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) + 0.013f);
    chassis_move_update->chassis_roll  = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET) + 0.003f;
    // ros数据
    // chassis_move_update->vx_from_ros = usb_fliter_data.vx_after_fliter;
    chassis_move_update->vx_from_ros = Usb_receive_data->vx_set;
    // chassis_move_update->wz_from_ros = usb_fliter_data.wz_after_fliter;
    // chassis_move_update->wz_from_ros = 0.5f;
    chassis_move_update->wz_from_ros = Usb_receive_data->wz_set;
    // printf("%3f %3f %3f\r\n",chassis_move_update->chassis_yaw,chassis_move_update->chassis_pitch,chassis_move_update->chassis_roll);
    // 更新关节电机角度
    //***********************************//
    chassis_move_update->right_leg.front_joint.angle = chassis_move_update->right_leg.front_joint.joint_motor_measure->pos;//phi4
    chassis_move_update->right_leg.back_joint.angle  = chassis_move_update->right_leg.back_joint.joint_motor_measure->pos;//phi1

    chassis_move_update->left_leg.front_joint.angle  = chassis_move_update->left_leg.front_joint.joint_motor_measure->pos;//phi4
    chassis_move_update->left_leg.back_joint.angle   = chassis_move_update->left_leg.back_joint.joint_motor_measure->pos;//phi1
    // printf("%.3f %.3f \r\n",chassis_move_update->right_leg.front_joint.angle,chassis_move_update->right_leg.back_joint.angle);
    // 更新关节转动速度
    //***********************************//
    chassis_move_update->right_leg.back_joint.angle_dot  = -chassis_move_update->right_leg.back_joint.joint_motor_measure->speed;
    chassis_move_update->right_leg.front_joint.angle_dot = -chassis_move_update->right_leg.front_joint.joint_motor_measure->speed;
    chassis_move_update->left_leg.back_joint.angle_dot   = chassis_move_update->left_leg.back_joint.joint_motor_measure->speed;
    chassis_move_update->left_leg.front_joint.angle_dot  = chassis_move_update->left_leg.front_joint.joint_motor_measure->speed;

    // VMC 计算腿部姿态
    fp32 L0_PHI[2];
    leg_pos(chassis_move_update->right_leg.back_joint.angle, chassis_move_update->right_leg.front_joint.angle, L0_PHI);
    chassis_move_update->right_leg.leg_angle  = L0_PHI[1];
    chassis_move_update->right_leg.leg_length = L0_PHI[0];

    leg_pos(chassis_move_update->left_leg.back_joint.angle, chassis_move_update->left_leg.front_joint.angle, L0_PHI);
    chassis_move_update->left_leg.leg_angle  = L0_PHI[1];
    chassis_move_update->left_leg.leg_length = L0_PHI[0];

    leg_spd(chassis_move_update->left_leg.back_joint.angle_dot, chassis_move_update->left_leg.front_joint.angle_dot,
            chassis_move_update->left_leg.back_joint.angle, chassis_move_update->left_leg.front_joint.angle, L0_PHI);
    chassis_move_update->left_leg.length_dot = L0_PHI[0];
    chassis_move_update->left_leg.angle_dot  = L0_PHI[1];

    leg_spd(chassis_move_update->right_leg.back_joint.angle_dot, chassis_move_update->right_leg.front_joint.angle_dot,
            chassis_move_update->right_leg.back_joint.angle, chassis_move_update->right_leg.front_joint.angle, L0_PHI);
    chassis_move_update->right_leg.length_dot = L0_PHI[0];
    chassis_move_update->right_leg.angle_dot  = L0_PHI[1];

    // 双腿状态量取平均即机器人腿部姿态(暂时没用到)
    chassis_move_update->leg_angle      = 0.5f * (chassis_move_update->right_leg.leg_angle + chassis_move_update->left_leg.leg_angle);
    chassis_move_update->leg_length     = 0.5f * (chassis_move_update->right_leg.leg_length + chassis_move_update->left_leg.leg_length);
    chassis_move_update->leg_length_dot = 0.5f * (chassis_move_update->right_leg.length_dot + chassis_move_update->left_leg.length_dot);
    chassis_move_update->leg_angle_dot  = 0.5f * (chassis_move_update->right_leg.angle_dot + chassis_move_update->left_leg.angle_dot);

    // 更新驱动轮电机速度，加速度是速度的PID微分
    //********************************************//
    // 线速度
    chassis_move_update->right_leg.wheel_motor.speed = -WHEEL_R * (float)chassis_move_update->right_leg.wheel_motor.wheel_motor_measure->speed / 57.3f;
    chassis_move_update->left_leg.wheel_motor.speed  = WHEEL_R * (float)chassis_move_update->left_leg.wheel_motor.wheel_motor_measure->speed / 57.3f;

    // 更新底盘旋转速度wz，坐标系为右手系
    ///////////////////////*****************************?????????????????/////////
    chassis_move_update->wz = 0.5f * (chassis_move_update->right_leg.wheel_motor.speed - chassis_move_update->left_leg.wheel_motor.speed) / MOTOR_DISTANCE_TO_CENTER;
    first_order_filter_cali(&chassis_move_update->state_xdot_filter, (chassis_move_update->right_leg.wheel_motor.speed + chassis_move_update->left_leg.wheel_motor.speed) * 0.5f);
    // 底盘状态量组装
    chassis_move_update->state_ref.theta     = chassis_move_update->leg_angle - PI / 2 - chassis_move_update->chassis_pitch; // 注意theta并不是腿与机体的夹角
    chassis_move_update->state_ref.theta_dot = chassis_move_update->leg_angle_dot + *(chassis_move_update->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET);
    // KF
    chassis_move_update->state_ref.x_dot = get_KF_Spd();

    // chassis_move_update->state_ref.x_dot = chassis_move_update->state_xdot_filter.out;

    if (chassis_move.position_control_intervention == true)//位置控制介入，底盘它会抵抗外界导致的位移并回正。
    {
        chassis_move_update->state_ref.x += 0.5f * (chassis_move_update->right_leg.wheel_motor.speed + chassis_move_update->left_leg.wheel_motor.speed) * CHASSIS_CONTROL_TIME;
        // chassis_move_update->state_ref.x +=  chassis_move_update->state_ref.x_dot* CHASSIS_CONTROL_TIME;
    }
        // chassis_move_update->state_ref.x += chassis_move_update->state_ref.x_dot * CHASSIS_CONTROL_TIME;
    else if (chassis_move.position_control_intervention == false&&(chassis_move.jump_state.jump_stage==0))//非跳跃阶段，位置控制不介入，只做平衡点补偿，底盘自由漂移，只跟随速度期望
    {

        chassis_move_update->state_ref.x = STOP_X_OFFSET;
    }
    else if (chassis_move.position_control_intervention == false&&(chassis_move.jump_state.jump_stage!=0))//跳跃阶段，位置控制不介入，只做平衡点补偿，
    {
        chassis_move_update->state_ref.x = STOP_X_OFFSET;
    }


    chassis_move_update->state_ref.phi     = chassis_move_update->chassis_pitch;
    chassis_move_update->state_ref.phi_dot = -*(chassis_move_update->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET);

    chassis_move_update->kilometer += 0.5f * (chassis_move_update->right_leg.wheel_motor.speed + chassis_move_update->left_leg.wheel_motor.speed) * CHASSIS_CONTROL_TIME;
    // 机器人离地判断
    // Robot_Offground_detect(chassis_move_update);
    // chassis_move_update->tmp = chassis_move_update->touchingGroung;
    // 机器人离地判断
    chassis_move_update->touchingGroung = Robot_Offground_detect(chassis_move_update);

}
/**
 * @brief          ͨ设置遥控器设置状态
 * @author         pxx
 * @param          chassis_move_mode   底盘结构体指针
 * @retval         void
 */
void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL) {
        return;
    }

    // chassis_behaviour_mode_set(chassis_move_mode);
    // 遥控器设置行为模式
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL])) {
        // 右上
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
    } else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL])) {
        // 右中
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    } else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL])) {
        // 右下
        chassis_move_mode->chassis_mode = CHASSIS_FORCE_RAW;
    }
}
/**
 * @brief          遥控器状态切换数据保存
 * @author         pxx
 * @param          chassis_move_transit    底盘结构体指针
 * @retval         void
 */
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL) {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) {
        return;
    }

    // 切入跟随云台模式(暂为开启lqr控制)
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        // chassis_move_transit->state_ref.x = 0;
        chassis_move_transit->state_set.x = 0;
        chassis_move_transit->state_ref.x = STOP_X_OFFSET;
    }
    // 切入不跟随云台模式(暂为开启腿长控制)
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
        // chassis_move_transit->state_ref.x = 0;
        chassis_move_transit->state_set.x = 0;
    }
    // 切入无力模式，清空里程计
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_FORCE_RAW) && chassis_move_transit->chassis_mode == CHASSIS_FORCE_RAW) {
        // chassis_move_transit->state_ref.x = 0;
        // chassis_move_transit->state_set.x = 0;

        chassis_move_transit->left_support_force  = 0.0f;
        chassis_move_transit->right_support_force = 0.0f;
        chassis_move_transit->leg_length_set      = LEG_LENGTH_INIT;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}
/**
 * @brief          遥控器的数据处理成底盘的前进vx速度，vy速度
 * @author         pxx
 * @param          vx_set  x轴前进速度设置，m/s
 * @param          vy_set  y轴前进速度设置，m/s
 * @param          chassis_move_rc_to_vector   底盘结构体指针
 * @retval         void
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *add_yaw_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || add_yaw_set == NULL) {
        return;
    }
    // 遥控器原始通道值
    int16_t vx_channel, wz_channel;
    fp32 vx_set_channel, add_yaw_channel;
    uint8_t now_toward;
    // 死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    // vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;

    ///////////////////////ROS////////////////////////////////////////////
    if (wz_channel != 0)
        add_yaw_channel = wz_channel * -CHASSIS_WZ_RC_SEN;
    else
        add_yaw_channel = chassis_move_rc_to_vector->wz_from_ros * CHASSIS_ROS_TO_WZ;

    if (vx_channel != 0)
        vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    else
        vx_set_channel = chassis_move_rc_to_vector->vx_from_ros;
    ///////////////////////////////////////ROS/////////////////////

    // 一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

    if (chassis_move_rc_to_vector->state_ref.x_dot > 0.05f)
        now_toward = CHASSIS_MOVE_AHEAD;
    else
        now_toward = CHASSIS_MOVE_BACK;

    // 停止信号，不需要缓慢加速，直接减速到零,速度设置
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) {
        chassis_move_rc_to_vector->mit_normal_kd               = MIT_KD_BRAKE;
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;

        if (!chassis_move_rc_to_vector->change_toward_flag) {
            chassis_move_rc_to_vector->chassis_move_toward = now_toward;
            chassis_move_rc_to_vector->change_toward_flag  = true;

            if (chassis_move_rc_to_vector->state_ref.x_dot < 0.25f && chassis_move_rc_to_vector->state_ref.x_dot > -0.25f) {
                chassis_move_rc_to_vector->position_control_intervention = true;
            }
        }

        if (chassis_move_rc_to_vector->change_toward_flag == true && now_toward != chassis_move_rc_to_vector->chassis_move_toward) {
            chassis_move_rc_to_vector->position_control_intervention = true;
        }
        // chassis_move_rc_to_vector->position_control_intervention = true;
    } else
        chassis_move_rc_to_vector->change_toward_flag = false;

    // yaw设置
    if (add_yaw_channel < CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN && add_yaw_channel > -CHASSIS_RC_DEADLINE * CHASSIS_WZ_RC_SEN) {
        add_yaw_channel = 0.0f;
    }

    *vx_set      = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *add_yaw_set = add_yaw_channel;
}
/**
 * @brief          遥控器的数据处理成轮腿底盘腿长、roll角
 * @author         pxx
 * @param          l_set  腿长设置
 * @param          roll_set  roll角设置
 * @param          chassis_move_rc_to_vector   底盘结构体指针
 * @retval         void
 */
void chassis_rc_to_control_euler(fp32 *l_set, fp32 *roll_set, chassis_move_t *chassis_move_rc_to_euler)
{
    fp32 add_l, add_l_channel;
    rc_deadline_limit(chassis_move_rc_to_euler->chassis_RC->rc.ch[CHASSIS_L_CHANNEL], add_l_channel, CHASSIS_RC_DEADLINE);
    add_l = add_l_channel * 0.0000006f;
    *l_set += add_l;

    fp32 roll_channel;
    rc_deadline_limit(chassis_move_rc_to_euler->chassis_RC->rc.ch[CHASSIS_ROLL_CHANNEL], roll_channel, CHASSIS_RC_DEADLINE);
    *roll_set = -roll_channel * 0.000264f;
}

// 腿长限制
//  腿部运动范围限制
void chassis_leg_limit(chassis_move_t *chassis_move_control, fp32 l_set)
{
    chassis_move_control->leg_length_set = l_set;

    if (chassis_move_control->leg_length_set > LEG_LENGTH_MAX)
        chassis_move_control->leg_length_set = LEG_LENGTH_MAX;
    else if (chassis_move_control->leg_length_set < LEG_LENGTH_MIN)
        chassis_move_control->leg_length_set = LEG_LENGTH_MIN;
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
/**
 * @brief          设置遥控器输入控制量
 * @author         pxx
 * @param          chassis_move_control    底盘结构体指针
 * @retval         void
 */
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL) {
        return;
    }

    // 设置速度
    fp32 vx_set = 0.0f, l_set = 0.0f, angle_set = 0.0f, roll_set = 0.0f;
    // chassis_behaviour_control_set(&vx_set, &l_set, &angle_set, chassis_move_control);
    // 当前腿长
    l_set = chassis_move_control->leg_length_set;
    //
    chassis_move_control->jump_state.jump_flag = false;

    if (switch_is_down(chassis_move_control->chassis_RC->rc.s[1])) // 左边拨到最下档才允许摇杆操控，防止不小心误触
    {
        chassis_move_control->mit_normal_kd = MIT_KD_NORMAL;
        chassis_rc_to_control_vector(&vx_set, &angle_set, chassis_move_control);
        chassis_rc_to_control_euler(&l_set, &roll_set, chassis_move_control);
        if (roll_set != 0.0f) {
            chassis_move_control->mit_normal_kd = MIT_KD_CXK;
        }
    } else if (switch_is_up(chassis_move_control->chassis_RC->rc.s[1])) // 跳跃，左边最上档
    {
        chassis_move_control->jump_state.jump_flag = true;
    } else {
        // 其余情况不设置速度和腿长
        chassis_move_control->mit_normal_kd = MIT_KD_NORMAL;
        vx_set                              = 0.0f;
        angle_set                           = 0.0f;
        roll_set                            = 0.0f;
    };

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW) {
        // 离地时不允许旋转
        if (chassis_move_control->right_leg.touching_ground == false || chassis_move_control->left_leg.touching_ground == false) {
            angle_set                             = 0;
            chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
        }

        chassis_move_control->chassis_yaw_set  = rad_format(chassis_move_control->chassis_yaw_set + angle_set);
        chassis_move_control->chassis_roll_set = roll_set;
        chassis_leg_limit(chassis_move_control, l_set);

        chassis_move_control->state_set.phi       = 0.0f;
        chassis_move_control->state_set.phi_dot   = 0.0f;
        chassis_move_control->state_set.theta     = 0.0f;
        chassis_move_control->state_set.theta_dot = 0.0f;

        // static uint16_t T_count = 1000;

        if (vx_set != 0) // 遥控器有输入
        {
            chassis_move_control->position_control_intervention = false; // 不进行位置控制
            chassis_move_control->state_set.x_dot               = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
            // chassis_move_control->state_set.x += chassis_move_control->state_set.x_dot * CHASSIS_CONTROL_TIME;
            chassis_move_control->state_set.x = 0.0f;

        } else // 停止立即刹车
        {
            // if(chassis_move_control->state_ref.x_dot <0.2f && chassis_move_control->state_ref.x_dot >  -0.2f)
            //     chassis_move_control->position_control_intervention = true;

            chassis_move_control->state_set.x_dot = 0.0f;
            chassis_move_control->state_set.x     = 0.0f;
        }
    } else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) {
        // 保持底盘与云台的相对角度不变//小陀螺试试
        if (switch_is_up(chassis_move_control->chassis_RC->rc.s[1])) {
            chassis_move_control->chassis_yaw_set += 0.008f;

        } else {
            chassis_move_control->chassis_yaw_set = chassis_move_control->chassis_yaw;
        }
    }
}
/**
 * @brief          支持力结算
 * @author         pxx
 * @param          chassis_move_control_loop   底盘结构体指针
 * @retval         void
 */
void cacul_support(chassis_move_t *chassis_move_control_loop)
{
    fp32 l_force = 0.0f, r_force = 0.0f;
    // 使用双腿长度的平均值，离地修改目标腿长

    if (chassis_move_control_loop->touchingGroung) // 正常触地状态
    {
        r_force = old_PID_Calc(&chassis_move_control_loop->right_leg_length_pid, chassis_move_control_loop->right_leg.leg_length, chassis_move_control_loop->right_leg.leg_length_set);
        l_force = old_PID_Calc(&chassis_move_control_loop->left_leg_length_pid, chassis_move_control_loop->left_leg.leg_length, chassis_move_control_loop->left_leg.leg_length_set);
        /*
        gravity_comp = (机体质量[除去轮子的质量]) × 9.8 / 2
                     = (11.983kg ) × 9.8 / 2
                     = 58.717 N
         */
        chassis_move_control_loop->l_force=l_force;
        chassis_move_control_loop->r_force=r_force;
        static const fp32 gravity_comp                 = 50.9f; // 补偿机体重力
        // static const fp32 gravity_comp                 = 64.9f; // 补偿机体重力
        // static const fp32 gravity_comp                 = 40.0f; // 补偿机体重力
        chassis_move_control_loop->left_support_force  = l_force + gravity_comp;
        chassis_move_control_loop->right_support_force = r_force + gravity_comp;

    } else {
        r_force = old_PID_Calc(&chassis_move_control_loop->right_leg_length_pid, chassis_move_control_loop->right_leg.leg_length, chassis_move_control_loop->right_leg.leg_length_in_sky_set);
        l_force = old_PID_Calc(&chassis_move_control_loop->left_leg_length_pid, chassis_move_control_loop->left_leg.leg_length, chassis_move_control_loop->left_leg.leg_length_in_sky_set);

        static const fp32 wheel_gravity_comp           = -(0.367f+0.963f)*10; // 补偿轮子重力
        // static const fp32 wheel_gravity_comp           = -3.0f; // 补偿轮子重力
        chassis_move_control_loop->left_support_force  = l_force + wheel_gravity_comp;
        chassis_move_control_loop->right_support_force = r_force + wheel_gravity_comp;
    }
}
/**
 * @brief          ROll 补偿
 * @author         pxx
 * @param          chassis_move_control_loop   底盘结构体指针
 * @retval         void
 */
void roll_compensate(chassis_move_t *chassis_move_control_loop)
{
    fp32 feed = 0.0f;

    fp32 roll_err = 0.0f;
    // PID补偿横滚角roll，这里作为腿长控制的外环，而不是直接补偿支持力
    if (chassis_move_control_loop->touchingGroung == true)
        roll_err = old_PID_Calc(&chassis_move_control_loop->roll_ctrl_l_pid, chassis_move_control_loop->chassis_roll, chassis_move_control_loop->chassis_roll_set);

    //腿长设置
    chassis_move_control_loop->tmp = roll_err;

    // 计算左右腿目标腿长
    fp32 left_target  = chassis_move_control_loop->leg_length_set + roll_err;
    fp32 right_target = chassis_move_control_loop->leg_length_set - roll_err;

    // 限幅检查:如果任意一腿超出物理限制,则限制roll_err并清除积分
    if (left_target < LEG_LENGTH_MIN || left_target > LEG_LENGTH_MAX ||
        right_target < LEG_LENGTH_MIN || right_target > LEG_LENGTH_MAX) {

        // 计算允许的最大roll_err
        fp32 max_roll_err_left  = (left_target < LEG_LENGTH_MIN) ? (LEG_LENGTH_MIN - chassis_move_control_loop->leg_length_set) : (LEG_LENGTH_MAX - chassis_move_control_loop->leg_length_set);
        fp32 max_roll_err_right = (right_target < LEG_LENGTH_MIN) ? -(LEG_LENGTH_MIN - chassis_move_control_loop->leg_length_set) : -(LEG_LENGTH_MAX - chassis_move_control_loop->leg_length_set);

        // 取较小的限制
        fp32 max_roll_err = (fabsf(max_roll_err_left) < fabsf(max_roll_err_right)) ? max_roll_err_left : max_roll_err_right;

        // 限制roll_err
        roll_err = max_roll_err;

        // 清除PID积分项,防止继续累积
        chassis_move_control_loop->roll_ctrl_l_pid.Iout = 0.0f;

        // 重新计算目标腿长
        left_target  = chassis_move_control_loop->leg_length_set + roll_err;
        right_target = chassis_move_control_loop->leg_length_set - roll_err;
    }

    chassis_move_control_loop->left_leg.leg_length_set  = left_target;
    chassis_move_control_loop->right_leg.leg_length_set = right_target;

    // 最终限幅保护
    if (chassis_move_control_loop->left_leg.leg_length_set < LEG_LENGTH_MIN)
        chassis_move_control_loop->left_leg.leg_length_set = LEG_LENGTH_MIN;
    if (chassis_move_control_loop->left_leg.leg_length_set > LEG_LENGTH_MAX)
        chassis_move_control_loop->left_leg.leg_length_set = LEG_LENGTH_MAX;
    if (chassis_move_control_loop->right_leg.leg_length_set < LEG_LENGTH_MIN)
        chassis_move_control_loop->right_leg.leg_length_set = LEG_LENGTH_MIN;
    if (chassis_move_control_loop->right_leg.leg_length_set > LEG_LENGTH_MAX)
        chassis_move_control_loop->right_leg.leg_length_set = LEG_LENGTH_MAX;

    // 原有的支持力补偿逻辑(保持不变)
    // chassis_move_control_loop->left_support_force -= feed;
    // chassis_move_control_loop->right_support_force += feed;
}

void Jump_v_coordinate(const chassis_move_t *chassis,float v_compente_tor[4])
{
    if (chassis->jump_state.jump_stage==0)
        return;
    float v_leg_left  = chassis->left_leg.length_dot;
    float v_leg_right = chassis->right_leg.length_dot;

    float erorr_v=v_leg_left - v_leg_right;//速度误差>0，说明左腿伸展速度大于右腿，左腿应该减速，右腿应该加速
    // if (chassis->jump_state.jump_stage==2)
    // {
        v_compente_tor[0] =-erorr_v*2.0f;
        v_compente_tor[1] = erorr_v*2.0f;
        v_compente_tor[2] = erorr_v*2.0f;
        v_compente_tor[3] =-erorr_v*2.0f;
    abs_limit(&v_compente_tor[0],2.0f);
    abs_limit(&v_compente_tor[1],2.0f);
    abs_limit(&v_compente_tor[2],2.0f);
    abs_limit(&v_compente_tor[3],2.0f);
    // }
    // else if (chassis->jump_state.jump_stage==3)
    // {
    //     v_compente_tor[0] =-erorr_v*2.0f;
    //     v_compente_tor[1] = erorr_v*2.0f;
    //     v_compente_tor[2] = erorr_v*2.0f;
    //     v_compente_tor[3] =-erorr_v*2.0f;
    // }
    // else if (chassis->jump_state.jump_stage==4)
    // {
    //     v_compente_tor[0]= erorr_v*2.0f;
    //     v_compente_tor[1]= erorr_v*2.0f;
    //     v_compente_tor[2]= erorr_v*2.0f;
    //     v_compente_tor[3]= erorr_v*2.0f;
    // }

}

void Jump_control(chassis_move_t *chassis_move_control_loop)
{
    static uint8_t jump_count = 0;          // 跳跃次数（0=从未跳跃，≥1=已跳跃过）
    chassis_move_control_loop->jump_state.current_time = DWT_GetTimeline_ms();// 当前时间（DWT高精度计时）
    uint8_t current_jump_flag = chassis_move_control_loop->jump_state.jump_flag; // 当前周期指令

    //双腿速度协调控制补偿力
    float v_compente_tor[4]={0.0f,0.0f,0.0f,0.0f};

    //阶段减速补偿力矩
    float stage_slow_com_tor[4]={0.0f,0.0f,0.0f,0.0f};

    //基本补偿力矩
     float tor0=2.0f;
     float tor1=0.0f;
     float tor2=3.0f;
     float tor3=1.4f;
     float tor4=2.4f;
     // float tor4=3.4f;
     // float tor0=2.0f;
     // float tor1=4.0f;
     // float tor2=5.0f;
     // float tor3=5.8f;
     // float tor4=5.0f;
    //
    /*计算跳跃高度的参数*/
     float L0[2]={0.0f,0.0f};//初始时的腿部高度
     float L1[2]={0.0f,0.0f};//伸腿后的腿部高度
     float h1[2]={0.0f,0.0f};//伸腿高度
     float h2[2]={0.0f,0.0f};//由于向上的速度带来的上升高度,h2=∑-V_L0*cos(theata)*▲t
     float h3[2]={0.0f,0.0f};//缩腿时间算出来的高度，h3=1/2*g*t3^2
     float h_total[2]={0.0f,0.0f};//总高度
     float jump_2_laste_time=0.0f;
     float jump_2_dlta_time=0.0f;
     float left_shrink_leg_time=0.0f;//缩腿时间
     float right_shrink_leg_time=0.0f;//缩腿时间
    bool left_cal_flag=true;
    bool right_cal_flag=true;
//    // ########## 核心：基于 last_jump_flag 的“边沿触发”判断 ##########
    // 1. 正在跳跃中（stage≠0）→ 忽略指令变化，不中断
    if (chassis_move_control_loop->jump_state.jump_stage != 0) {
        // 可选：跳跃中拨杆再次触发（1→1），直接清零flag，避免干扰
        if (current_jump_flag == 1) {
            chassis_move_control_loop->jump_state.jump_flag = 0;
        }
    }
    // 2. 未跳跃（stage=0）→ 仅响应“0→1”的上升沿（拨杆刚拨到跳跃位置）
    else {
        // 触发条件：当前=1，上一周期=0（刚按下拨杆）
        bool_t jump_trigger = (current_jump_flag == 1) && (chassis_move_control_loop->jump_state.last_jump_flag == 0);
        if (jump_trigger) {
            // 冷却时间判断（初次跳跃无冷却）
            bool_t can_jump = (jump_count == 0) ||
                             (chassis_move_control_loop->jump_state.current_time - chassis_move_control_loop->jump_state.last_jump_finish_time >= JUMP_COOLDOWN_MS);

            if (can_jump) {
            // 启动新跳跃
            jump_count++;
            chassis_move_control_loop->jump_state.jump_stage = 1;
            // 禁用位置介入控制，避免刹车
            chassis_move_control_loop->position_control_intervention = false;

            // chassis_move_control_loop->jump_state.takeoff_velocity = chassis_move_control_loop->state_ref.x_dot;
            // chassis_move_control_loop->jump_state.takeoff_pitch = chassis_move_control_loop->chassis_pitch;
            chassis_move_control_loop->jump_state.takeoff_leg_length = 0.5f * (chassis_move_control_loop->left_leg.leg_length +
            chassis_move_control_loop->right_leg.leg_length);
            }
            // 无论是否能跳，都清零当前flag，避免重复触发
            chassis_move_control_loop->jump_state.jump_flag = 0;
            // chassis_move_control_loop->jump_state.jump_comtorque[0]=-tor0;
            // chassis_move_control_loop->jump_state.jump_comtorque[1]= tor0;
            // chassis_move_control_loop->jump_state.jump_comtorque[2]=-tor0;
            // chassis_move_control_loop->jump_state.jump_comtorque[3]= tor0;
        }
        // 未触发时，保持初始状态
        chassis_move_control_loop->left_leg.leg_length_in_sky_set = LEG_LENGTH_INIT;
        chassis_move_control_loop->right_leg.leg_length_in_sky_set = LEG_LENGTH_INIT;
        chassis_move_control_loop->jump_state.jump_comtorque[0]= 0;
        chassis_move_control_loop->jump_state.jump_comtorque[1]= 0;
        chassis_move_control_loop->jump_state.jump_comtorque[2]= 0;
        chassis_move_control_loop->jump_state.jump_comtorque[3]= 0;
    }

    // ########## 更新上一周期指令状态（必须放在最后） ##########
    chassis_move_control_loop->jump_state.last_jump_flag = current_jump_flag;

    /////////state_change_end//////////////////
    // 禁用位置介入控制，避免刹车
    if (chassis_move_control_loop->jump_state.jump_stage != 0)
    chassis_move_control_loop->position_control_intervention = false;
    /////////jump_start//////////////////
   //双腿速度误差控制器
    Jump_v_coordinate(chassis_move_control_loop,v_compente_tor);
    // 准备起跳，缩腿
    if (chassis_move_control_loop->jump_state.jump_stage == 1) {
        chassis_move_control_loop->right_leg.leg_length_set = LEG_LENGTH_MIN;
        chassis_move_control_loop->left_leg.leg_length_set  = LEG_LENGTH_MIN;

        chassis_move_control_loop->jump_state.jump_comtorque[0]=-tor1;//左前电机
        chassis_move_control_loop->jump_state.jump_comtorque[1]= tor1;//左后电机
        chassis_move_control_loop->jump_state.jump_comtorque[2]=-tor1;//右后电机
        chassis_move_control_loop->jump_state.jump_comtorque[3]= tor1;//右前电机
        // if (chassis_move_control_loop->right_leg.leg_length < 0.125f && chassis_move_control_loop->left_leg.leg_length < 0.125f) {
        if ( chassis_move_control_loop->left_leg.leg_length < 0.125f) {
            chassis_move_control_loop->jump_state.jump_stage = 2;
            chassis_move_control_loop->jump_state.takeoff_start_time= DWT_GetTimeline_ms();
            // 记录起跳时的水平初速度
            chassis_move_control_loop->jump_state.takeoff_velocity_x = chassis_move_control_loop->state_ref.x_dot;
            //
            L0[0]=chassis_move_control_loop->left_leg.leg_length;
            L0[1]=chassis_move_control_loop->right_leg.leg_length;
            chassis_move_control_loop->left_leg_length_pid.Iout=0.0f;
            chassis_move_control_loop->right_leg_length_pid.Iout=0.0f;
        }

    }
    // 起跳
    if (chassis_move_control_loop->jump_state.jump_stage == 2) {

        float takeoff_elapsed = chassis_move_control_loop->jump_state.current_time - chassis_move_control_loop->jump_state.takeoff_start_time; // 起跳阶段经过时间
        // chassis_move_control_loop->leg_length_in_sky = 0.35f;
        chassis_move_control_loop->left_leg.leg_length_in_sky_set = 0.35f;
        chassis_move_control_loop->right_leg.leg_length_in_sky_set = 0.35f;

        // /*计算上升高度*/
        jump_2_dlta_time=chassis_move_control_loop->jump_state.current_time-jump_2_laste_time;// 当前时间（DWT高精度计时）
        jump_2_laste_time=chassis_move_control_loop->jump_state.current_time;
        float left_theata=chassis_move_control_loop->left_leg.leg_angle - PI / 2 - chassis_move_control_loop->chassis_pitch;
        float right_theata=chassis_move_control_loop->right_leg.leg_angle - PI / 2 - chassis_move_control_loop->chassis_pitch;
        // h2[0]+=(chassis_move_control_loop->left_leg.length_dot*cosf(left_theata)*jump_2_dlta_time*0.001f);;//由于向上的速度带来的上升高度,h2=∑-V_L0*cos(theata)*▲t
        // h2[1]+=(chassis_move_control_loop->right_leg.length_dot*cosf(right_theata)*jump_2_dlta_time*0.001f);;//由于向上的速度带来的上升高度,h2=∑-V_L0*cos(theata)*▲t
        if (chassis_move_control_loop->left_leg.length_dot>0.0f)
        {
        h2[0]+=(chassis_move_control_loop->left_leg.length_dot*cosf(left_theata)*jump_2_dlta_time*0.001f);;//由于向上的速度带来的上升高度,h2=∑-V_L0*cos(theata)*▲t
        }
        if (chassis_move_control_loop->right_leg.length_dot>0.0f)
        {
        h2[1]+=(chassis_move_control_loop->right_leg.length_dot*cosf(right_theata)*jump_2_dlta_time*0.001f);;//由于向上的速度带来的上升高度,h2=∑-V_L0*cos(theata)*▲t
        }

        // 目标腿长=初始腿长（0.166）+ 伸腿幅度（0.18m）→ 总0.346m（略超0.33m切换阈值，确保能进入第三阶段）
        float extend_target = 0.35f;
        chassis_move_control_loop->right_leg.leg_length_set = extend_target; // 给右腿伸腿目标
        chassis_move_control_loop->left_leg.leg_length_set = extend_target;  // 给左腿伸腿目标


        float left_leg_over = chassis_move_control_loop->left_leg.leg_length - (0.34f - 0.02f); // 提前2cm开始减速
        float right_leg_over = chassis_move_control_loop->right_leg.leg_length - (0.34f - 0.02f); // 提前2cm开始减速

        if (left_leg_over > 0.0f) {
           // tor2=tor2-1.0f;
            stage_slow_com_tor[0]=-0.9f;
            stage_slow_com_tor[1]= 0.9f;
        }
        if (right_leg_over > 0.0f) {
            stage_slow_com_tor[2]=-0.9f;
            stage_slow_com_tor[3]= 0.9f;
        }

        chassis_move_control_loop->jump_state.jump_comtorque[0]= tor2+stage_slow_com_tor[0]+v_compente_tor[0];
        chassis_move_control_loop->jump_state.jump_comtorque[1]=-tor2+stage_slow_com_tor[1]+v_compente_tor[1];
        chassis_move_control_loop->jump_state.jump_comtorque[2]= tor2+stage_slow_com_tor[2]+v_compente_tor[2];
        chassis_move_control_loop->jump_state.jump_comtorque[3]=-tor2+stage_slow_com_tor[3]+v_compente_tor[3];
        if (chassis_move_control_loop->right_leg.leg_length > 0.34f && chassis_move_control_loop->left_leg.leg_length > 0.34f) {
        // if ( chassis_move_control_loop->left_leg.leg_length > 0.34f) {
            chassis_move_control_loop->jump_state.jump_stage = 3;
            chassis_move_control_loop->jump_state.takeoff_time = DWT_GetTimeline_ms();
            // chassis_move_control_loop->leg_length_in_sky=0.15f;
            chassis_move_control_loop->left_leg.leg_length_in_sky_set =  0.15f;
            chassis_move_control_loop->right_leg.leg_length_in_sky_set = 0.15f;
            chassis_move_control_loop->left_leg.leg_length_set  = 0.15f;
            chassis_move_control_loop->right_leg.leg_length_set = 0.15f;
            // 清零PID积分：避免起跳阶段积分积累，导致空中收腿延迟
            // chassis_move_control_loop->left_leg_length_pid.Iout = 0.0f;
            // chassis_move_control_loop->right_leg_length_pid.Iout = 0.0f;
            //
            left_cal_flag=true;
            right_cal_flag=true;
            L1[0]=chassis_move_control_loop->left_leg.leg_length;
            L1[1]=chassis_move_control_loop->right_leg.leg_length;

            chassis_move_control_loop->left_leg_length_pid.Iout=0.0f;
            chassis_move_control_loop->right_leg_length_pid.Iout=0.0f;
        }

    }
    // 收腿（空中阶段）
    if (chassis_move_control_loop->jump_state.jump_stage == 3) {

        uint32_t time_in_sky = DWT_GetTimeline_ms() - chassis_move_control_loop->jump_state.takeoff_time;


        /*计算h1*/
        h1[0]=L1[0]-L0[0];//伸腿后的腿部高度
        h1[1]=L1[1]-L0[1];//伸腿后的腿部高度
        /*计算上升高度*/


        // jump_2_dlta_time=chassis_move_control_loop->jump_state.current_time-jump_2_laste_time;// 当前时间（DWT高精度计时）
        // jump_2_laste_time=chassis_move_control_loop->jump_state.current_time;
        // float left_theata=chassis_move_control_loop->left_leg.leg_angle - PI / 2 - chassis_move_control_loop->chassis_pitch;
        // float right_theata=chassis_move_control_loop->right_leg.leg_angle - PI / 2 - chassis_move_control_loop->chassis_pitch;

        /*计算h3*/
        if (chassis_move_control_loop->left_leg.leg_length<0.12f&&left_cal_flag)
        {
            left_shrink_leg_time=(chassis_move_control_loop->jump_state.current_time - chassis_move_control_loop->jump_state.takeoff_time)*0.001f;;//缩腿时间
            h3[0]=0.5f*9.8f*left_shrink_leg_time*left_shrink_leg_time;;//缩腿时间算出来的高度，h3=1/2*g*t3^2
            // h_total[0]=h1[0]+h3[0];//总高度
            h_total[0]=h1[0]+h2[0]+h3[0];//总高度
            left_cal_flag=false;
        }
        if (chassis_move_control_loop->right_leg.leg_length<0.12f&&right_cal_flag)
        {
            right_shrink_leg_time=(chassis_move_control_loop->jump_state.current_time - chassis_move_control_loop->jump_state.takeoff_time)*0.001f;;//缩腿时间
            h3[1]=0.5f*9.8f*right_shrink_leg_time*right_shrink_leg_time;;//缩腿时间算出来的高度，h3=1/2*g*t3^2
            // h_total[1]=h1[1]+h3[1];//总高度
            h_total[1]=h1[1]+h2[1]+h3[1];//总高度
            right_cal_flag=false;
        }
        if (h_total[0]>(L0[0]+WHEEL_R)||h_total[1]>(L0[1]+WHEEL_R))
        {
            chassis_move_control_loop->jump_state.jump_height[0]=h_total[0] -L0[0]-WHEEL_R;//
            chassis_move_control_loop->jump_state.jump_height[1]=h_total[1] -L0[1]-WHEEL_R;//
        }
        // 关键1：根据空中时间和水平初速度，计算理论水平位移
        float vx = chassis_move_control_loop->jump_state.takeoff_velocity_x;// 起跳时水平速度
        float horizontal_displacement = vx * (time_in_sky / 1000.0f); // 水平位移=速度×时间

        // 2. 核心：通过 state_set.theta 控制摆杆前伸角度（摆杆=theta）
        // theta 定义：摆杆与竖直方向的夹角（0=竖直向下，正值=摆杆向前摆，负值=向后摆）
        // 位移越大，前伸角度越大（最大30°=PI/6，避免机械限位）
        // float target_theta = fp32_constrain(horizontal_displacement * 1.5, -0.2f, 0.52f); // 2倍系数可调，最大约30度(0.52rad)

        // 摆杆角度PID控制（复用angle_err_pid，跟踪state_set.theta）
        // fp32 theta_error = target_theta - chassis_move_control_loop->state_ref.theta;
        // fp32 theta_pid_out =theta_error*5.0f - chassis_move_control_loop->state_ref.theta_dot * 1.2f;
        // chassis_move_control_loop->state_set.theta = theta_pid_out;

        // 固定空中目标腿长为0.15m，确保不会因腿长过大导致落地奇异位
        // fp32 target_leg_len = 0.15f;
        // chassis_move_control_loop->leg_length_in_sky = target_leg_len;



        // if (chassis_move_control_loop->left_leg.length_dot < -1.8f) { // 最大允许伸展速度1.8m/s
        //     if (chassis_move_control_loop->jump_state.left_target_comp<0.0f)
        //     chassis_move_control_loop->jump_state.left_target_comp = -chassis_move_control_loop->jump_state.left_target_comp*0.75f;
        // }
        // if (chassis_move_control_loop->right_leg.length_dot < -1.8f) { // 最大允许伸展速度1.8m/s
        //     if (chassis_move_control_loop->jump_state.right_target_comp<0.0f)
        //     chassis_move_control_loop->jump_state.right_target_comp = -chassis_move_control_loop->jump_state.right_target_comp*0.75f;
        // }
        // --------------- 4. 腿长下限保护（强制不碰机械限位）---------------

        // // 收腿力限幅（确保足够收缩力，同时避免电机过载）
        // chassis_move_control_loop->left_support_force  += chassis_move_control_loop->jump_state.left_target_comp-l_compente;
        // chassis_move_control_loop->right_support_force += chassis_move_control_loop->jump_state.right_target_comp+l_compente;
        // chassis_move_control_loop->left_support_force  = fp32_constrain(chassis_move_control_loop->left_support_force ,-300.0f, 150.0f);
        // chassis_move_control_loop->right_support_force = fp32_constrain(chassis_move_control_loop->right_support_force, -300.0f, 150.0f);
        // float left_leg_over  = chassis_move_control_loop->left_leg.leg_length  - (0.15f); // 提前2cm开始减速
        // float right_leg_over = chassis_move_control_loop->right_leg.leg_length - (0.15f); // 提前2cm开始减速

        //减小力矩绝对值
        if ( chassis_move_control_loop->left_leg.length_dot < -1.6f) {
            stage_slow_com_tor[0] = tor3;
            stage_slow_com_tor[1] =-tor3;
            // stage_slow_com_tor[0] = 5.9f;
            // stage_slow_com_tor[1] =-5.9f;
            // if (chassis_move_control_loop->left_leg.leg_length<0.18)
            chassis_move_control_loop->left_leg.leg_length_in_sky_set  = chassis_move_control_loop->left_leg.leg_length;
            // chassis_move_control_loop->left_leg.leg_length_set  = chassis_move_control_loop->left_leg.leg_length;
        }
        if ( chassis_move_control_loop->right_leg.length_dot < -1.6f) {
            stage_slow_com_tor[2] = tor3;
            stage_slow_com_tor[3] =-tor3;
            // stage_slow_com_tor[2] = 5.9f;
            // stage_slow_com_tor[3] =-5.9f;
            // if (chassis_move_control_loop->right_leg.leg_length<0.18)
            chassis_move_control_loop->right_leg.leg_length_in_sky_set =  chassis_move_control_loop->right_leg.leg_length;
        }

        chassis_move_control_loop->jump_state.jump_comtorque[0]=-tor3+stage_slow_com_tor[0]+v_compente_tor[0];
        chassis_move_control_loop->jump_state.jump_comtorque[1]= tor3+stage_slow_com_tor[1]+v_compente_tor[1];
        chassis_move_control_loop->jump_state.jump_comtorque[2]=-tor3+stage_slow_com_tor[2]+v_compente_tor[2];
        chassis_move_control_loop->jump_state.jump_comtorque[3]= tor3+stage_slow_com_tor[3]+v_compente_tor[3];

        bool_t compelete_shrink = (chassis_move_control_loop->left_leg.leg_length<0.15&&chassis_move_control_loop->right_leg.leg_length<0.15); // 完成收腿;
        bool_t time_out_sky = (time_in_sky > 350); // 超时保护
        if (compelete_shrink || time_out_sky) {
        // if (chassis_move_control_loop->left_leg.leg_length<0.15) {
            chassis_move_control_loop->jump_state.jump_stage = 4;
            chassis_move_control_loop->jump_state.landing_time = DWT_GetTimeline_ms();
            chassis_move_control_loop->left_leg.leg_length_set  = 0.20f;
            chassis_move_control_loop->right_leg.leg_length_set = 0.20f;
            chassis_move_control_loop->left_leg.leg_length_in_sky_set  = 0.20f;
            chassis_move_control_loop->right_leg.leg_length_in_sky_set = 0.20f;
            chassis_move_control_loop->state_set.theta= 0.0f; // 落地后摆杆回正
            // 记录落地瞬间收腿力，用于缓冲阶段力衔接
            // chassis_move_control_loop->jump_state.landing_left_force = chassis_move_control_loop->left_support_force;
            // chassis_move_control_loop->jump_state.landing_right_force = chassis_move_control_loop->right_support_force;

            // 清零腿长PID积分，避免落地力突变
            chassis_move_control_loop->left_leg_length_pid.Iout = 0.0f;
            chassis_move_control_loop->right_leg_length_pid.Iout = 0.0f;

        }
    }
    //  关键修改：落地缓冲阶段（stage=4）
    if (chassis_move_control_loop->jump_state.jump_stage == 4) {
        uint32_t time_since_landing = DWT_GetTimeline_ms() - chassis_move_control_loop->jump_state.landing_time;

        /*计算h3*/
        // h3[0]=0.5f*9.8f*(time_since_landing*0.001f)*(time_since_landing*0.001f);//缩腿时间算出来的高度，h3=1/2*g*t3^2

        chassis_move_control_loop->jump_state.jump_comtorque[0]= tor4+v_compente_tor[0];
        chassis_move_control_loop->jump_state.jump_comtorque[1]=-tor4+v_compente_tor[1];
        chassis_move_control_loop->jump_state.jump_comtorque[2]= tor4+v_compente_tor[2];
        chassis_move_control_loop->jump_state.jump_comtorque[3]=-tor4+v_compente_tor[3];
        // 退出条件：缓冲稳定后回到初始状态
        // bool_t time_enough = (time_since_landing > 600);
        bool_t leg_stable =  (fabsf(chassis_move_control_loop->left_leg.length_dot) < 0.02f) &&
                           (fabsf(chassis_move_control_loop->right_leg.length_dot) < 0.02f);
        bool_t force_stable = (fabsf(chassis_move_control_loop->left_support_force - 60.0f) < 10.0f) &&
                            (fabsf(chassis_move_control_loop->right_support_force - 60.0f) < 10.0f);
        // bool_t attitude_stable = (fabsf(chassis_move_control_loop->chassis_pitch) < 0.05f); // 姿态稳定（<2.9°）
        // if (time_enough && leg_stable && force_stable && attitude_stable) {
        if ( leg_stable &&force_stable) {
        // if (chassis_move_control_loop->left_leg.leg_length>0.15&&chassis_move_control_loop->right_leg.leg_length>0.15&&chassis_move_control_loop->touchingGroung) {
            chassis_move_control_loop->jump_state.jump_stage = 0;
            chassis_move_control_loop->jump_state.jump_flag = false;
            chassis_move_control_loop->jump_state.jump_comtorque[0]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[1]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[2]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[3]=0.0f;
            chassis_move_control_loop->left_leg.leg_length_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->right_leg.leg_length_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->left_leg.leg_length_in_sky_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->right_leg.leg_length_in_sky_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->jump_state.last_jump_finish_time = chassis_move_control_loop->jump_state.current_time; // 记录本次跳跃完成时间

            chassis_move_control_loop->left_leg_length_pid.Iout=0.0f;
            chassis_move_control_loop->right_leg_length_pid.Iout=0.0f;
        }
        // 超时强制退出，防止卡壳
        if (time_since_landing > 350) {
            chassis_move_control_loop->jump_state.jump_stage = 0;
            chassis_move_control_loop->jump_state.jump_flag = false;
            chassis_move_control_loop->left_leg.leg_length_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->right_leg.leg_length_set = LEG_LENGTH_INIT;
            chassis_move_control_loop->jump_state.jump_comtorque[0]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[1]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[2]=0.0f;
            chassis_move_control_loop->jump_state.jump_comtorque[3]=0.0f;
            chassis_move_control_loop->jump_state.last_jump_finish_time = chassis_move_control_loop->jump_state.current_time; // 记录完成时间

            chassis_move_control_loop->left_leg_length_pid.Iout=0.0f;
            chassis_move_control_loop->right_leg_length_pid.Iout=0.0f;
        }
    }
}
static float limitted_motor_current(fp32 current, fp32 max)
{
    if (fabsf(current) > max) {
        current = current > 0 ? max : -max;
    }
    return (float)current;
}

float calc_lambda(float slip_conf)
{
    if (slip_conf <= 0.5f) return 1.0f;
    if (slip_conf >= 0.65f) return 0.0f;

    float x = (slip_conf - 0.5f) / (0.65f - 0.5f); // 0~1
    return 0.5f * (1.0f + cosf(M_PI * x));
}
float calc_I_limit(float slip_conf)
{
    // 0.2 以下认为正常
    if (slip_conf <= 0.5f)
        return 2000;

    // 0.65 以上强打滑
    if (slip_conf >= 0.60f)
        return 1400;

    float x = (slip_conf - 0.5f) / (0.60f - 0.5f);
    float w = 0.5f * (1.0f + cosf(M_PI * x)); // 1 -> 0

    return 1400 +
           w * (2000 - 1400);
}

/**
 * @brief       平衡+转向
 * @author
 * @param
 *
 * @retval  获得leg_tor和wheel_tor ,即T-p和T
 */
void LQR_Balance_Turn(chassis_move_t *chassis_move_control_loop)
{
    fp32 yaw_err_force      = 0.0f;
    fp32 yaw_force_from_ros = 0.0f;
    //状态变量x=[theata, theta_dot, x, x_dot, phi, phi_dot]T
    //u=[T,Tp]^T

    // 反馈矩阵乘以一定系数用于调整
    fp32 coefficient[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
                              {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
    fp32 slip_coefficient[2][6] = {{0.7f, 0.6f, 0.9f, 0.7f, 0.8f, 0.7f},
                              {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
    fp32 jump_4_coefficient[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 0.9f, 0.8f},
                                     {1.4f, 1.2f, 1.0f, 1.0f, 1.5f, 1.3f}};
    fp32 jump_3_coefficient[2][6] = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
                                     {1.5f, 1.3f, 0.0f, 0.0f, 1.8f, 1.5f}};
    fp32 jump_2_coefficient[2][6] = {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f},
                                     {1.5f, 1.3f, 1.0f, 1.0f, 1.5f, 1.3f}};
    fp32 kRes[12] = {0}, k[2][6] = {0};
    // 输入腿长，函数把K矩阵放在k中
    lqr_k(chassis_move_control_loop->leg_length, kRes);
    //[0    2    4   ..              ]
    //[1    3    5   ..              ]
    if (chassis_move_control_loop->touchingGroung&&(chassis_move_control_loop->jump_state.jump_stage==0)) // 正常触地、不跳跃状态
    {
        // if (slip_flag==SLIP_NONE)
        // {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * coefficient[j][i];
        // }
        // else
        // {
        //     for (int i = 0; i < 6; i++)
        //         for (int j = 0; j < 2; j++)
        //             k[j][i] = kRes[i * 2 + j] * slip_coefficient[j][i];
        // }


    }
    else if (chassis_move_control_loop->jump_state.jump_stage==2) // 起跳阶段
    {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * jump_2_coefficient[j][i];
    }
    else if (chassis_move_control_loop->jump_state.jump_stage==3)
    {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * jump_3_coefficient[j][i];
    }
    else if (chassis_move_control_loop->jump_state.jump_stage==4&&chassis_move_control_loop->touchingGroung==false)
    {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * jump_3_coefficient[j][i];
    }
    else if (chassis_move_control_loop->jump_state.jump_stage==4&&chassis_move_control_loop->touchingGroung)
    {
        for (int i = 0; i < 6; i++)
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * jump_4_coefficient[j][i];
    }
    else // 腿部离地状态，手动修改反馈矩阵，仅保持腿部竖直
    {
        memset(k, 0, sizeof(k));
        //原地起跳参数
        // k[1][0] = kRes[1] * 1.5f;// pitch角反馈系数（加大）
        // k[1][1] = kRes[3] * 1.3f; // pitch角速度反馈系数（加大）
        k[1][0] = kRes[1] * 1.5f;// theata角反馈系数（加大）
        k[1][1] = kRes[3] * 1.2f; // theata角速度反馈系数（加大）

        k[1][4]=kRes[9]*1.3;  //phi角反馈系数
        k[1][5]=kRes[11]*1.2; //phi角速度反馈系数
    }
    // 设置向量
    // 控制率中的u=k(L0)(xd-x)
    // LQR,得到轮力矩和关节力矩
    fp32 x[6] = {chassis_move_control_loop->state_set.theta - chassis_move_control_loop->state_ref.theta,
                 chassis_move_control_loop->state_set.theta_dot - chassis_move_control_loop->state_ref.theta_dot,
                 chassis_move_control_loop->state_set.x - chassis_move_control_loop->state_ref.x,
                 chassis_move_control_loop->state_set.x_dot - chassis_move_control_loop->state_ref.x_dot,
                 chassis_move_control_loop->state_set.phi - chassis_move_control_loop->state_ref.phi,
                 chassis_move_control_loop->state_set.phi_dot - chassis_move_control_loop->state_ref.phi_dot};

    chassis_move_control_loop->wheel_tor = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
    chassis_move_control_loop->leg_tor   = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];

    chassis_move_control_loop->wheel_tor *= 0.5f; // 两条腿，每条腿只取一半
    chassis_move_control_loop->leg_tor *= 0.5f;

    // printf("%3f %3f \r\n",x[0],x[1]);
    //  PID计算转向 左右轮力矩差 注意过零保护
    // 位置环，速度环
    chassis_move_control_loop->wz_set = old_PID_Calc(&chassis_move_control_loop->chassis_angle_pid, rad_format(chassis_move_control_loop->chassis_yaw - chassis_move_control_loop->chassis_yaw_set), 0);
    // chassis_move_control_loop->wz_set = chassis_move_control_loop->wz_from_ros;
    yaw_err_force = old_PID_Calc(&chassis_move_control_loop->chassis_yaw_gyro_pid, *(chassis_move_control_loop->chassis_imu_gyro + INS_GYRO_Z_ADDRESS_OFFSET), chassis_move_control_loop->wz_set);
    float conf_l, conf_r,lambda_l, lambda_r;
    conf_l=get_confidence_left();
    conf_r=get_confidence_right();
    // lambda_l=calc_lambda(conf_l);
    // lambda_r=calc_lambda(conf_r);

    float I_cmd_l = (chassis_move_control_loop->wheel_tor - yaw_err_force) * LK9025_TOR_TO_CAN_DATA;
    float I_cmd_r = -(chassis_move_control_loop->wheel_tor + yaw_err_force) * LK9025_TOR_TO_CAN_DATA;

    float I_lim_l = calc_I_limit(conf_l);
    float I_lim_r = calc_I_limit(conf_r);

    I_cmd_l = limitted_motor_current(I_cmd_l,I_lim_l);
    I_cmd_r = limitted_motor_current(I_cmd_r,I_lim_r);

    // 遥控器么有输入时候，可以用ROS的wz
    //  if( chassis_move_control_loop->wz_from_ros != 0.0f ){
    //      //chassis_move_control_loop->wz_from_ros = 0.3f;
    //      //printf("?");
    //      yaw_err_force = old_PID_Calc(&chassis_move_control_loop->chassis_ros_wz_pid,*(chassis_move_control_loop->chassis_imu_gyro+INS_GYRO_Z_ADDRESS_OFFSET),chassis_move_control_loop->wz_from_ros);
    //  }

    // 单腿离地之后轮子不能转，给关节加上kd阻尼
    if (chassis_move_control_loop->touchingGroung) {
        // chassis_move_control_loop->right_leg.wheel_motor.give_current = I_cmd_r;
        // chassis_move_control_loop->left_leg.wheel_motor.give_current  = I_cmd_l;
        chassis_move_control_loop->right_leg.wheel_motor.give_current = limitted_motor_current(-(chassis_move_control_loop->wheel_tor + yaw_err_force) * LK9025_TOR_TO_CAN_DATA, 2000);
        chassis_move_control_loop->left_leg.wheel_motor.give_current  = limitted_motor_current((chassis_move_control_loop->wheel_tor - yaw_err_force) * LK9025_TOR_TO_CAN_DATA, 2000);

        if (chassis_move_control_loop->now_tick - chassis_move_control_loop->last_out_ground_tick > 1000) {
            chassis_move_control_loop->right_leg.mit_kd = chassis_move_control_loop->mit_normal_kd;
            chassis_move_control_loop->left_leg.mit_kd  = chassis_move_control_loop->mit_normal_kd;
        } else {
            chassis_move_control_loop->right_leg.mit_kd = MIT_KD_OUTGROUND;
            chassis_move_control_loop->left_leg.mit_kd  = MIT_KD_OUTGROUND;
        }
    } else if (chassis_move_control_loop->right_leg.touching_ground == false && chassis_move_control_loop->is_conversely == false) {
        chassis_move_control_loop->left_leg.wheel_motor.give_current = 0.0f;

        chassis_move_control_loop->left_leg.mit_kd = MIT_KD_OUTGROUND;
    } else if (chassis_move_control_loop->left_leg.touching_ground == false && chassis_move_control_loop->is_conversely == false) {
        chassis_move_control_loop->right_leg.wheel_motor.give_current = 0.0f;

        chassis_move_control_loop->right_leg.mit_kd = MIT_KD_OUTGROUND;
    } else {
        chassis_move_control_loop->right_leg.wheel_motor.give_current = 0.0f;
        chassis_move_control_loop->left_leg.wheel_motor.give_current  = 0.0f;

        chassis_move_control_loop->right_leg.mit_kd = MIT_KD_OUTGROUND;
        chassis_move_control_loop->left_leg.mit_kd  = MIT_KD_OUTGROUND;
    }
}
void chassis_joint_tor_cal(chassis_move_t * chassis)
{
    // 双腿角度误差控制
    fp32 err_tor   = 0.0f;
    fp32 angle_dot = old_PID_Calc(&chassis->angle_err_pid, (chassis->right_leg.leg_angle - chassis->left_leg.leg_angle), 0);
    err_tor        = old_PID_Calc(&chassis->angle_dot_pid, (chassis->right_leg.angle_dot - chassis->left_leg.angle_dot), angle_dot);
    chassis->err_tor=err_tor;

    //打滑机体关节电机力矩补偿
    float comp_left  = 0.0f;
    float comp_right = 0.0f;
    LegPredictor_GetCompensation(&comp_left, &comp_right);

    // VMC 虚拟力解算
    fp32 tor_vector[2] = {0.0f};

    // 右腿VMC：基础leg_tor + 右腿补偿 + err_tor
    leg_conv(-chassis->right_support_force, chassis->leg_tor +comp_right+ err_tor,
            chassis->right_leg.back_joint.angle, chassis->right_leg.front_joint.angle, \
            tor_vector);
    //VMC计算出的力矩叠加跳跃补偿力矩
    //注意-号
    chassis->tor_vector[2]=-tor_vector[1]+chassis->jump_state.jump_comtorque[2];
    chassis->tor_vector[3]=-tor_vector[0]+chassis->jump_state.jump_comtorque[3];
    // if (chassis_move_control_loop->jump_state.jump_stage==0)
    // {
        chassis->right_leg.back_joint.tor_set = limitted_motor_current( chassis->tor_vector[2] , 18.0f);
        chassis->right_leg.front_joint.tor_set = limitted_motor_current(chassis->tor_vector[3] , 18.0f);
    // }
    // else
    // {
        // chassis_move_control_loop->right_leg.back_joint.tor_set = limitted_motor_current(-tor_vector[1] , 20.0f);
        // chassis_move_control_loop->right_leg.front_joint.tor_set = limitted_motor_current(-tor_vector[0] , 20.0f);
    // }


    leg_conv(-chassis->left_support_force, chassis->leg_tor +comp_left- err_tor,
             chassis->left_leg.back_joint.angle, chassis->left_leg.front_joint.angle,
             tor_vector);
    //VMC计算出的力矩叠加跳跃补偿力矩
    chassis->tor_vector[0]=tor_vector[0]+chassis->jump_state.jump_comtorque[0];
    chassis->tor_vector[1]=tor_vector[1]+chassis->jump_state.jump_comtorque[1];
    // if (chassis_move_control_loop->jump_state.jump_stage==0)
    // {
        chassis->left_leg.back_joint.tor_set  = limitted_motor_current(chassis->tor_vector[1], 18.0f);
        chassis->left_leg.front_joint.tor_set = limitted_motor_current(chassis->tor_vector[0], 18.0f);
    // }
    // else
    // {
        // chassis_move_control_loop->left_leg.back_joint.tor_set  = limitted_motor_current(tor_vector[1], 20.0f);
        // chassis_move_control_loop->left_leg.front_joint.tor_set = limitted_motor_current(tor_vector[0], 20.0f);

    // }
}
/**
 * @brief          底盘控制PID计算
 * @author         pxx
 * @param          chassis_move_control_loop   底盘结构体指针
 * @retval         void
 */
void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    if (chassis_move_control_loop->chassis_mode == CHASSIS_FORCE_RAW) {
        chassis_move_control_loop->right_leg.front_joint.tor_set = 0;
        chassis_move_control_loop->right_leg.back_joint.tor_set  = 0;
        chassis_move_control_loop->left_leg.front_joint.tor_set  = 0;
        chassis_move_control_loop->left_leg.back_joint.tor_set   = 0;

        chassis_move_control_loop->right_leg.wheel_motor.give_current = 0;
        chassis_move_control_loop->left_leg.wheel_motor.give_current  = 0;

        chassis_move_control_loop->is_conversely = true;

        chassis_move_control_loop->state_ref.x     = STOP_X_OFFSET;
        chassis_move_control_loop->state_ref.x_dot = 0.0f;

        chassis_move_control_loop->kilometer = 0.0f;
        // printf("fuck \r\n");
        return;
    }

        // 获取沿杆的推力F,给出左右leg_force
        cacul_support(chassis_move_control_loop);

        // 通过ROLL轴补偿腿长以及推力F，离地则不补偿,给出leg_force和补偿过的leg_set
        roll_compensate(chassis_move_control_loop);

        // 跳跃
        Jump_control(chassis_move_control_loop);  //

        // LQR平衡,轮子力矩和推力Tp
        LQR_Balance_Turn(chassis_move_control_loop);



        chassis_joint_tor_cal(chassis_move_control_loop);

}

// 机器人支持力解算，判断离地
static bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect)
{
    static fp32 last_length_dot = 0.0f; // 差分计算腿长变化的加速度
    static fp32 last_theta_dot  = 0.0f; // 差分计算腿部倾角变化加速度
    // 新增：状态切换计时和计数，用于过滤噪声
    static uint32_t offground_timer = 0;
    static uint32_t touchdown_timer = 0;
    //轮子总重1.33kg
    static const fp32 m_w = 0.367f+0.963f; // 轮毂的质量：0.300f->0.367f
    static const fp32 g   = 9.8f;   // 重力加速度
    // 对腿长速度、倾角速度做低通滤波（抑制高频噪声）
    static fp32 filtered_length_dot = 0.0f;
    static fp32 filtered_theta_dot  = 0.0f;
    filtered_length_dot = 0.2f * chassis_move_detect->leg_length_dot + (1 - 0.2f) * filtered_length_dot;
    filtered_theta_dot  = 0.2f * chassis_move_detect->state_ref.theta_dot  + (1 - 0.2f) * filtered_theta_dot;
    // printf("%f\r\n",p_zw_ddot);
    //  // 由于使用了双腿误差控制，双腿角度一致，只计算一次三角函数
    fp32 cos_theta = arm_cos_f32(chassis_move_detect->state_ref.theta);
    fp32 sin_theta = arm_sin_f32(chassis_move_detect->state_ref.theta);

    // // 直接使用上一控制周期计算的力矩作为力矩反馈值
    // fp32 leg_support_force = (chassis_move_detect->left_support_force + chassis_move_detect->right_support_force);
    // 使用差分代替微分运算
    fp32 ddlength = (filtered_length_dot - last_length_dot) / CHASSIS_CONTROL_TIME;
    // fp32 ddtheta  = (chassis_move_detect->state_ref.theta_dot - last_theta_dot) / CHASSIS_CONTROL_TIME;
    fp32 ddtheta  = (filtered_theta_dot - last_theta_dot) / CHASSIS_CONTROL_TIME;
    // chassis_move_detect->state_ref.theta_dot=filtered_theta_dot;
    // chassis_move_detect->state_ref.theta_ddot=ddtheta;
    // ddlength = 0.0f;// 噪声太大了，需要滤波，于是暂时没有使用
    // ddtheta = 0.0f;
    last_length_dot = filtered_length_dot;
    last_theta_dot = filtered_theta_dot;
    // last_length_dot = chassis_move_detect->leg_length_dot;
    // last_theta_dot = chassis_move_detect->state_ref.theta_dot;
    // 额外：对加速度再做一次滑动平均（可选，噪声极大时启用）
    static fp32 ddlength_buf[3] = {0}; // 3点滑动平均
    static fp32 ddtheta_buf[3]  = {0};
    ddlength_buf[2] = ddlength_buf[1];
    ddlength_buf[1] = ddlength_buf[0];
    ddlength_buf[0] = ddlength;
    ddlength = (ddlength_buf[0] + ddlength_buf[1] + ddlength_buf[2]) / 3.0f;

    ddtheta_buf[2] = ddtheta_buf[1];
    ddtheta_buf[1] = ddtheta_buf[0];
    ddtheta_buf[0] = ddtheta;
    ddtheta = (ddtheta_buf[0] + ddtheta_buf[1] + ddtheta_buf[2]) / 3.0f;
    chassis_move_detect->state_ref.theta_ddot=ddtheta;
    // 更新历史值
    last_length_dot = filtered_length_dot;
    last_theta_dot  = filtered_theta_dot;
    // // 机器人腿部机构作用于驱动轮竖直向下的力
    fp32 Pl = chassis_move_detect->left_support_force * cos_theta + chassis_move_detect->leg_tor * sin_theta / chassis_move_detect->left_leg.leg_length;
    fp32 Pr = chassis_move_detect->right_support_force * cos_theta + chassis_move_detect->leg_tor * sin_theta / chassis_move_detect->right_leg.leg_length;
    // 驱动轮竖直方向运动加速度
    fp32 w_acc_z_l =
                    -ddlength * cos_theta
                     + 2.0f * chassis_move_detect->left_leg.length_dot *  chassis_move_detect->state_ref.theta_dot * sin_theta
                     + chassis_move_detect->left_leg.leg_length * ddtheta * sin_theta
                     + chassis_move_detect->left_leg.leg_length * chassis_move_detect->state_ref.theta_dot * chassis_move_detect->state_ref.theta_dot * cos_theta;

    fp32 w_acc_z_r =
                    -ddlength * cos_theta
                     + 2.0f * chassis_move_detect->right_leg.length_dot * chassis_move_detect->state_ref.theta_dot * sin_theta
                     + chassis_move_detect->right_leg.leg_length * ddtheta * sin_theta
                     + chassis_move_detect->right_leg.leg_length * chassis_move_detect->state_ref.theta_dot*chassis_move_detect->state_ref.theta_dot * cos_theta;

    fp32 P_left_real, P_right_real;

    P_left_real  = Pl + m_w * (w_acc_z_l + g);
    P_right_real = Pr + m_w * (w_acc_z_r + g);

    // --------------- 5. 支持力再滤波（最终平滑） ---------------
    static fp32 filtered_left_force = 0.0f;
    static fp32 filtered_right_force = 0.0f;
    filtered_left_force  = 0.2f * P_left_real  + (1 - 0.2f) * filtered_left_force;
    filtered_right_force = 0.2f * P_right_real + (1 - 0.2f) * filtered_right_force;

    // 赋值给全局变量（用滤波后的值）
    chassis_move_detect->right_leg_real_support = filtered_right_force;
    chassis_move_detect->left_leg_real_support  = filtered_left_force;
    chassis_move_detect->ground_force           = filtered_left_force + filtered_right_force;
    // chassis_move_detect->right_leg_real_support = P_right_real;
    // chassis_move_detect->left_leg_real_support  = P_left_real;
    // chassis_move_detect->ground_force           = P_right_real + P_left_real;
    // 使用双阈值的方式进行判断，防止机器人在离地与触地之间反复切换

    /////////////
    chassis_move_detect->now_tick = xTaskGetTickCount();
    bool_t current_touching = chassis_move_detect->touchingGroung;

    // 1. 结合跳跃阶段辅助判断：跳跃阶段3（空中）强制优先判断为离地
    if (chassis_move_detect->jump_state.jump_stage == 3) {
        current_touching = false;
        offground_timer = 0; // 重置计时，避免干扰
        touchdown_timer = 0;
    } else {
        // 2. 基于地面总力的滞回阈值判断
        const fp32 TOUCHDOWN_THRESHOLD = 20.0f;  // 触地阈值（提高，减少误判）
        const fp32 OFFGROUND_THRESHOLD = -10.0f; // 离地阈值（放宽，避免早判）
        const uint32_t STABLE_DELAY = 5;         // 稳定时间（控制周期数）

        if (current_touching) {
            // 当前为触地状态：检测是否持续满足离地条件
            if (chassis_move_detect->ground_force < OFFGROUND_THRESHOLD) {
                offground_timer++;
                if (offground_timer >= STABLE_DELAY) {
                    current_touching = false; // 持续满足，切换为离地
                    chassis_move_detect->last_out_ground_tick = chassis_move_detect->now_tick;
                    offground_timer = 0;
                }
            } else {
                offground_timer = 0; // 不满足，重置计时
            }
        } else {
            // 当前为离地状态：检测是否持续满足触地条件
            if (chassis_move_detect->ground_force > TOUCHDOWN_THRESHOLD) {
                touchdown_timer++;
                if (touchdown_timer >= STABLE_DELAY) {
                    current_touching = true; // 持续满足，切换为触地
                    chassis_move_detect->chassis_yaw_set = chassis_move_detect->chassis_yaw;
                    touchdown_timer = 0;
                }
            } else {
                touchdown_timer = 0; // 不满足，重置计时
            }
        }
    }
    // if (chassis_move_detect->touchingGroung == true && chassis_move_detect->ground_force < -20.0f) {
    //     chassis_move_detect->touchingGroung       = false;
    //     chassis_move_detect->last_out_ground_tick = xTaskGetTickCount();
    //
    // } else if (chassis_move_detect->touchingGroung == false && chassis_move_detect->ground_force > 15.0f) {
    //     chassis_move_detect->touchingGroung  = true;
    //     chassis_move_detect->chassis_yaw_set = chassis_move_detect->chassis_yaw; // 落地后保证朝向
    // }

    // //倒地
    if (chassis_move_detect->touchingGroung == true)
        chassis_move_detect->is_conversely = false;

    if (chassis_move_detect->touchingGroung == false && chassis_move_detect->is_conversely == true)
        chassis_move_detect->touchingGroung = true;
    chassis_move_detect->touchingGroung = current_touching;
    return chassis_move_detect->touchingGroung;
}
void jump_test(chassis_move_t *chassis)
{
    float tor0=2.0f;
    float tor1=4.0f;
    float tor2=6.0f;
    float tor3=7.0f;
    float tor4=7.0f;
    if (chassis->jump_state.jump_stage==0)
    {
        chassis->jump_state.jump_comtorque[0]=-tor0;
        chassis->jump_state.jump_comtorque[1]= tor0;
        chassis->jump_state.jump_comtorque[2]=-tor0;
        chassis->jump_state.jump_comtorque[3]= tor0;
    }
    else if (chassis->jump_state.jump_stage==1)
    {
        chassis->jump_state.jump_comtorque[0]=-tor1;//左前电机
        chassis->jump_state.jump_comtorque[1]= tor1;//左后电机
        chassis->jump_state.jump_comtorque[2]=-tor1;//右后电机
        chassis->jump_state.jump_comtorque[3]= tor1;//右前电机
    }
    else if (chassis->jump_state.jump_stage==2)
    {
        chassis->jump_state.jump_comtorque[0]= tor2;
        chassis->jump_state.jump_comtorque[1]=-tor2;
        chassis->jump_state.jump_comtorque[2]= tor2;
        chassis->jump_state.jump_comtorque[3]=-tor2;
    }
    else if (chassis->jump_state.jump_stage==3)
    {
        chassis->jump_state.jump_comtorque[0]=-tor3;
        chassis->jump_state.jump_comtorque[1]= tor3;
        chassis->jump_state.jump_comtorque[2]=-tor3;
        chassis->jump_state.jump_comtorque[3]= tor3;
    }
    else if (chassis->jump_state.jump_stage==4)
    {
        chassis->jump_state.jump_comtorque[0]= tor4;
        chassis->jump_state.jump_comtorque[1]=-tor4;
        chassis->jump_state.jump_comtorque[2]= tor4;
        chassis->jump_state.jump_comtorque[3]=-tor4;
    }

}
// 获取底盘结构体指针
const chassis_move_t *get_chassis_control_point(void)
{
    return &chassis_move;
}
