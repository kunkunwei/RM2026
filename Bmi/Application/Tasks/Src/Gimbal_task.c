#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
// #include "shoot.h"

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

gimbal_t gimbal;
void init_gimbal_shoot(gimbal_shoot_t* shoot);
// void cal_gimbal_shoot_control(gimbal_shoot_t* shoot_control);

// void init_gimbal_pos_control(gimbal_pos_control_t* pos_control);
//
void init_gimbal_control(gimbal_t* gimbal);
void gimbal_control_loop(gimbal_t* gimbal);
//
void limit_current(gimbal_t* gimbal);
//
void gimbal_calibrate_control(gimbal_t* gimbal);
void gimbal_follow_chassis_control(gimbal_t* gimbal);
void gimbal_no_follow_chassis_control(gimbal_t* gimbal);
//
void gimbal_rc_set_control(gimbal_t* gimbal,float *add_yaw,float *add_pitch);

void Gimbal_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);
    TickType_t systick = 0;
    //shoot_init();
    init_gimbal_control(&gimbal);
    // gimbal.gimbal_shoot.shoot_ready_flag = true;
    // init_gimbal_shoot(&gimbal.gimbal_shoot);
    for(;;)
    {   
        systick = osKernelSysTick();
        gimbal_behaviour_mode_set(gimbal.gimbal_RC);

        //cal_gimbal_shoot_control(&gimbal.gimbal_shoot);

        gimbal_control_loop(&gimbal);

        limit_current(&gimbal);
        //printf("%d\r\n",given_current);

        osDelayUntil(&systick,2);
    }
}

void init_gimbal_shoot(gimbal_shoot_t* shoot){

    const static fp32 pull_w_pid[3] =    {PID_SHOOT_PULL_MOTOR_KP,PID_SHOOT_PULL_MOTOR_KI,PID_SHOOT_PULL_MOTOR_KD};
    const static fp32 shoot_w_r_pid[3] = {PID_SHOOT_RIGHT_MOTOR_KP,PID_SHOOT_RIGHT_MOTOR_KI,PID_SHOOT_RIGHT_MOTOR_KD};
    const static fp32 shoot_w_l_pid[3] = {PID_SHOOT_LEFT_MOTOR_KP,PID_SHOOT_LEFT_MOTOR_KI,PID_SHOOT_LEFT_MOTOR_KD};

    old_PID_Init(&shoot->Shoot_right_PID, PID_POSITION, shoot_w_r_pid,PID_SHOOT_RIGHT_MOTOR_MAXOUT, PID_SHOOT_RIGHT_MOTOR_MAXI);
    old_PID_Init(&shoot->Shoot_left_PID, PID_POSITION, shoot_w_l_pid,PID_SHOOT_LEFT_MOTOR_MAXOUT, PID_SHOOT_LEFT_MOTOR_MAXI);
    old_PID_Init(&shoot->Pull_PID,      PID_POSITION,        pull_w_pid,PID_SHOOT_PULL_MOTOR_MAXOUT, PID_SHOOT_PULL_MOTOR_MAXI);

    shoot->pull_motor = get_shoot_motor_pull();
    shoot->shoot_motor_right = get_shoot_motor_right();
    shoot->shoot_motor_left = get_shoot_motor_left();

}
void cal_gimbal_shoot_control(gimbal_shoot_t* shoot_control,float shoot_spd,bool_t shoot_flag,bool_t fire_flag){

    fp32 i=1;
    i=(gimbal.gimbal_RC->rc.ch[0]+1)/100.0f; //拨蛋速度，RC0通道控制拨蛋速度，范围
    // if (switch_is_up(gimbal.gimbal_RC->rc.s[0]))
    // {
    //     k=-k;
    // }
    if(shoot_flag)
    {
        old_PID_Calc(&shoot_control->Shoot_left_PID,shoot_control->shoot_motor_left->real_w,-SHOOT_SPD);
        old_PID_Calc(&shoot_control->Shoot_right_PID,shoot_control->shoot_motor_right->real_w,SHOOT_SPD);
        // old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,-2*3.1415f* 2.0f);
    }
    else{
        old_PID_clear(&shoot_control->Shoot_left_PID);
        old_PID_clear(&shoot_control->Shoot_right_PID);
        old_PID_clear(&shoot_control->Pull_PID);

        old_PID_Calc(&shoot_control->Shoot_left_PID,shoot_control->shoot_motor_left->real_w,0.0f);
        old_PID_Calc(&shoot_control->Shoot_right_PID,shoot_control->shoot_motor_right->real_w,0.0f);
        // old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,0.0f);
    }

    //初始力比较大，防止以及有弹丸堵住
    if(shoot_control->shoot_motor_left->real_w < -SHOOT_SPD/2.0f && shoot_control->shoot_motor_right->real_w < SHOOT_SPD/2.0f)
    {
        //Dji_Motor_Shoot_Can_Send(3000 + Shoot_l_PID.out, -4000 + Shoot_r_PID.out,0);
        shoot_control->Shoot_right_PID.Kp = PID_SHOOT_LOCK_KP;
        shoot_control->Shoot_left_PID.Kp  = PID_SHOOT_LOCK_KP;
    }
    else
    {
        shoot_control->Shoot_right_PID.Kp = PID_SHOOT_RIGHT_MOTOR_KP;
        shoot_control->Shoot_left_PID.Kp = PID_SHOOT_LEFT_MOTOR_KP;
    }
    
    // //摩擦轮速度到达最大时，READY，此时可以拨蛋
    // if(shoot_control->shoot_motor_left->real_w < -SHOOT_SPD && shoot_control->shoot_motor_right->real_w > SHOOT_SPD)
    // {
    //     shoot_control->shoot_ready_flag = true;
    // }
    // else
    //     shoot_control->shoot_ready_flag = false;
    shoot_control->shoot_ready_flag = true;

    if (switch_is_up(gimbal.gimbal_RC->rc.s[1]))
    {
        if (gimbal.gimbal_RC->rc.ch[3]<10&&gimbal.gimbal_RC->rc.ch[3]>-10)
        {
            old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,0.0f);

        }
        else if (gimbal.gimbal_RC->rc.ch[3]>=10||gimbal.gimbal_RC->rc.ch[3]<=-10)
        {
            old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,gimbal.gimbal_RC->rc.ch[3]*i);
            // shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
        }
        shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
    }
    else if (switch_is_down(gimbal.gimbal_RC->rc.s[1])&&(gimbal.gimbal_RC->rc.ch[4]>400))
    {
        old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_pos,3.1415f*1.5f);
        shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
    }
    else
    {
        old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_pos,0);
        shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
    }
    //
    // if(shoot_control->shoot_ready_flag == true && fire_flag ==true)
    // {
    //     shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
    //     fire_flag=!fire_flag; //每次拨蛋后，切换拨蛋状态
    // }
    // else
    // {
    //     shoot_control->pull_motor->target_current = 0;
    //     // fire_flag=!fire_flag; //每次拨蛋后，切换拨蛋状态
    // }
    // shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;

}


void init_gimbal_pos_control(gimbal_pos_control_t* pos_control){

    const static fp32 gimbal_yaw_order_filter[1] = {0.166f};
    const static fp32 gimbal_pitch_order_filter[1] = {0.166f};
    first_order_filter_init(&pos_control->filter_rc_yaw_vel_set, 0.002f, gimbal_yaw_order_filter);
    first_order_filter_init(&pos_control->filter_rc_pitch_vel_set, 0.002f, gimbal_pitch_order_filter);

    //const static fp32 pitch_vel_pid[3] =    {PID_PITCH_MOTOR_VEL_KP,PID_PITCH_MOTOR_VEL_KI,PID_PITCH_MOTOR_VEL_KD};
    const static fp32 pitch_pos_pid[3] =    {PID_PITCH_MOTOR_POS_KP,PID_PITCH_MOTOR_POS_KI,PID_PITCH_MOTOR_POS_KD};

    //const static fp32 yaw_vel_pid[3] =    {PID_Y_MOTOR_VEL_KP,PID_PITCH_MOTOR_VEL_KI,PID_PITCH_MOTOR_VEL_KD};
    const static fp32 yaw_pos_pid[3] =    {PID_YAW_MOTOR_POS_KP,PID_YAW_MOTOR_POS_KI,PID_YAW_MOTOR_POS_KD};

    //old_PID_Init(&pos_control->Pitch_Vel_PID, PID_POSITION, pitch_vel_pid,PID_PITCH_MOTOR_VEL_MAXOUT, PID_PITCH_MOTOR_VEL_MAXI);
    old_PID_Init(&pos_control->Pitch_Pos_PID, PID_POSITION, pitch_pos_pid,PID_PITCH_MOTOR_POS_MAXOUT, PID_PITCH_MOTOR_POS_MAXI);

    //old_PID_Init(&pos_control->Yaw_Vel_PID, PID_POSITION, pitch_yaw_pid,PID_YAW_MOTOR_VEL_MAXOUT, PID_YAW_MOTOR_VEL_MAXI);
    old_PID_Init(&pos_control->Yaw_Pos_PID, PID_POSITION, yaw_pos_pid,PID_YAW_MOTOR_POS_MAXOUT, PID_YAW_MOTOR_POS_MAXI);

    pos_control->yaw_target_pos = 0.0f;
    pos_control->pitch_target_pos = 0.0f;

    pos_control->pitch_motor = get_pitch_motor();
    pos_control->yaw_motor = get_yaw_motor();

    pos_control->calibrate_warning = false;
}   

void gimbal_calibrate_control(gimbal_t* gimbal){
    //pitch
    
    old_PID_Calc(&gimbal->gimbal_pos.Pitch_Pos_PID,gimbal->gimbal_pos.pitch_motor->real_pos,0.0f);

    gimbal->gimbal_pos.pitch_motor->target_current = (int16_t)(gimbal->gimbal_pos.Pitch_Pos_PID.out);
    //yaw
    old_PID_Calc(&gimbal->gimbal_pos.Yaw_Pos_PID,gimbal->gimbal_pos.yaw_motor->pos,0.0f);
    gimbal->gimbal_pos.yaw_motor->target_tor = gimbal->gimbal_pos.Yaw_Pos_PID.out;
}

void cal_gimbal_pos_control(gimbal_t* gimbal){
    if(gimbal == NULL)
    {
        return;
    }

    if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_DISABLE){
        gimbal->gimbal_pos.pitch_motor->target_current = 0;

        old_PID_clear(&gimbal->gimbal_pos.Pitch_Pos_PID);
        //old_PID_clear(&gimbal->gimbal_pos.Pitch_Vel_PID);
        gimbal->gimbal_pos.pitch_motor->target_current = 0;
        gimbal->gimbal_pos.yaw_motor->target_tor = 0.0f;

        gimbal->gimbal_pos.pitch_target_pos = 0.0f;
        gimbal->gimbal_pos.yaw_target_pos = 0.0f;

        return;
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_CALIBRATE){
        gimbal_calibrate_control(gimbal);
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_FOLLOW_CHASSIS){
        //控制量给到底盘，云台锁定正前方
        //gimbal_calibrate_control(gimbal);
        gimbal_follow_chassis_control(gimbal);
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_FREE_CONTROL){
        gimbal_no_follow_chassis_control(gimbal);
    }

}

void init_gimbal_control(gimbal_t* gimbal){
    init_gimbal_shoot(&gimbal->gimbal_shoot);
    init_gimbal_pos_control(&gimbal->gimbal_pos);

    gimbal->rc_control_mode = get_rc_control_mode();
    gimbal->gimbal_RC = get_remote_control_point();

}

void gimbal_control_loop(gimbal_t* gimbal){

    cal_gimbal_shoot_control(&gimbal->gimbal_shoot,\
                             gimbal->gimbal_RC->rc.ch[RC_ROLL_CH] * SHOOT_CHANNEL_TO_SPEED,\
                             gimbal->rc_control_mode->shoot_flag,\
                             gimbal->rc_control_mode->fire_flag);

    cal_gimbal_pos_control(gimbal);

}
void limit_current(gimbal_t* gimbal){

    if(gimbal->gimbal_shoot.pull_motor->target_current > 9000.0f)
        gimbal->gimbal_shoot.pull_motor->target_current=9000.0f;
    if(gimbal->gimbal_shoot.pull_motor->target_current < -9000.0f)
        gimbal->gimbal_shoot.pull_motor->target_current=-9000.0f;
}

void gimbal_rc_set_control(gimbal_t* gimbal,float *add_yaw,float *add_pitch){

    if (gimbal)
    {
        return;
    }
    //遥控器原始通道值

    int16_t wz_channel,pitch_channel;
    fp32 wz_set_channel,pitch_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0

    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_X_CH], wz_channel, 20);
    rc_deadline_limit(gimbal->gimbal_RC->rc.ch[RC_RIGHT_Y_CH], pitch_channel, 20);

    wz_set_channel = wz_channel * GIMBAL_WZ_RC_SEN;
    pitch_set_channel = pitch_channel * GIMBAL_PITCH_RC_SEN;

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_yaw_vel_set, wz_set_channel);
    first_order_filter_cali(&gimbal->gimbal_pos.filter_rc_pitch_vel_set, pitch_set_channel);

    *add_pitch = gimbal->gimbal_pos.filter_rc_pitch_vel_set.out;
    *add_yaw = gimbal->gimbal_pos.filter_rc_yaw_vel_set.out;

}

void gimbal_follow_chassis_control(gimbal_t* gimbal){
    float add_pitch,add_yaw;
    gimbal_rc_set_control(gimbal,&add_yaw,&add_pitch);

    //pitch
    gimbal->gimbal_pos.pitch_target_pos += add_pitch;
    old_PID_Calc(&gimbal->gimbal_pos.Pitch_Pos_PID,gimbal->gimbal_pos.pitch_motor->real_pos,gimbal->gimbal_pos.pitch_target_pos);

    gimbal->gimbal_pos.pitch_motor->target_current = (int16_t)(gimbal->gimbal_pos.Pitch_Pos_PID.out);
    //yaw 跟随底盘，保持yaw电机和底盘不发生转动即可
    //yaw
    old_PID_Calc(&gimbal->gimbal_pos.Yaw_Pos_PID,gimbal->gimbal_pos.yaw_motor->pos,0.0f);
    gimbal->gimbal_pos.yaw_motor->target_tor = gimbal->gimbal_pos.Yaw_Pos_PID.out;
}

void gimbal_no_follow_chassis_control(gimbal_t* gimbal){
    float add_pitch,add_yaw;
    gimbal_rc_set_control(gimbal,&add_yaw,&add_pitch);
    //pitch
    old_PID_Calc(&gimbal->gimbal_pos.Pitch_Pos_PID,gimbal->gimbal_pos.pitch_motor->real_pos,gimbal->gimbal_pos.pitch_target_pos);

    gimbal->gimbal_pos.pitch_motor->target_current = (int16_t)(gimbal->gimbal_pos.Pitch_Pos_PID.out);

    //yaw,要用IMU
    // old_PID_Calc(&gimbal->gimbal_pos.Yaw_Pos_PID,gimbal->gimbal_pos.yaw_motor->pos,gimbal->gimbal_pos.yaw_target_pos);
    // gimbal->gimbal_pos.yaw_motor->target_tor = gimbal->gimbal_pos.Yaw_Pos_PID.out;
}