#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
#include "shoot.h"
// #include "remote_control.h"

gimbal_t gimbal;
void init_gimbal_shoot(Gimbal_Shoot_t* shoot);

void init_gimbal_pos_control(gimbal_pos_control_t* pos_control);
//
void init_gimbal_control(gimbal_t* gimbal);
void gimbal_control_loop(gimbal_t* gimbal);
//
void limit_current(gimbal_t* gimbal);
//
void gimbal_calibrate_control(gimbal_t* gimbal);

//
void Gimbal_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);
    TickType_t systick = 0;
    shoot_init();
    init_gimbal_control(&gimbal);
    
    for(;;)
    {
        systick = osKernelSysTick();
        gimbal_behaviour_mode_set(gimbal.gimbal_RC);


        gimbal_control_loop(&gimbal);

        limit_current(&gimbal);

        
        osDelayUntil(&systick,2);
    }
}

void init_gimbal_shoot(Gimbal_Shoot_t* shoot){

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
void cal_gimbal_shoot_control(Gimbal_Shoot_t* shoot_control,float shoot_spd,bool_t shoot_flag,bool_t fire_flag){

    if(shoot_flag)
    {
        old_PID_Calc(&shoot_control->Shoot_left_PID,shoot_control->shoot_motor_left->real_w,-SHOOT_SPD);
        old_PID_Calc(&shoot_control->Shoot_right_PID,shoot_control->shoot_motor_right->real_w,SHOOT_SPD);
        old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,-2*3.1415f* 2.0f);
    }
    else{
        old_PID_clear(&shoot_control->Shoot_left_PID);
        old_PID_clear(&shoot_control->Shoot_right_PID);
        old_PID_clear(&shoot_control->Pull_PID);

        old_PID_Calc(&shoot_control->Shoot_left_PID,shoot_control->shoot_motor_left->real_w,0.0f);
        old_PID_Calc(&shoot_control->Shoot_right_PID,shoot_control->shoot_motor_right->real_w,0.0f);
        old_PID_Calc(&shoot_control->Pull_PID   ,   shoot_control->pull_motor->real_w,0.0f);
    }

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

    if(shoot_control->shoot_motor_left->real_w < -SHOOT_SPD && shoot_control->shoot_motor_right->real_w > SHOOT_SPD)
    {
        shoot_control->shoot_ready_flag = true;
    }
    else 
        shoot_control->shoot_ready_flag = false;

    shoot_control->shoot_motor_right->target_current = shoot_control->Shoot_right_PID.out;
    shoot_control->shoot_motor_left->target_current = shoot_control->Shoot_left_PID.out;

    if(shoot_control->shoot_ready_flag == true && fire_flag ==true)
        shoot_control->pull_motor->target_current = shoot_control->Pull_PID.out;
    else
        shoot_control->pull_motor->target_current = 0;
    
    /////////////SHOOT_END/////////////////////////////////////
}
void init_gimbal_pos_control(gimbal_pos_control_t* pos_control){
    const static fp32 pitch_vel_pid[3] =    {PID_PITCH_MOTOR_VEL_KP,PID_PITCH_MOTOR_VEL_KI,PID_PITCH_MOTOR_VEL_KD};
    const static fp32 pitch_pos_pid[3] =    {PID_PITCH_MOTOR_POS_KP,PID_PITCH_MOTOR_POS_KI,PID_PITCH_MOTOR_POS_KD};

    old_PID_Init(&pos_control->Pitch_Vel_PID, PID_POSITION, pitch_vel_pid,PID_PITCH_MOTOR_VEL_MAXOUT, PID_PITCH_MOTOR_VEL_MAXI);
    old_PID_Init(&pos_control->Pitch_Pos_PID, PID_POSITION, pitch_pos_pid,PID_PITCH_MOTOR_POS_MAXOUT, PID_PITCH_MOTOR_POS_MAXI);

    pos_control->pitch_motor = get_pitch_motor();
    pos_control->yaw_motor = get_gimbal_motor();

    pos_control->calibrate_warning = false;
}   

void gimbal_calibrate_control(gimbal_t* gimbal){
    //pitch
    
    //weizhihuan
    old_PID_Calc(&gimbal->gimbal_pos.Pitch_Pos_PID,gimbal->gimbal_pos.pitch_motor->real_pos,0.0f);
    //suduhuan
    //fp32 pitch_vel = gimbal->gimbal_pos.Pitch_Pos_PID.out;
    //fp32 pitch_vel = 0.1f;
    //old_PID_Calc(&gimbal->gimbal_pos.Pitch_Vel_PID,gimbal->gimbal_pos.pitch_motor->real_w,pitch_vel);

    gimbal->gimbal_pos.pitch_motor->target_current = (int16_t)(gimbal->gimbal_pos.Pitch_Pos_PID.out);
    //yaw
}

void cal_gimbal_pos_control(gimbal_t* gimbal){
    if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_DISABLE){
        gimbal->gimbal_pos.pitch_motor->target_current = 0;

        old_PID_clear(&gimbal->gimbal_pos.Pitch_Pos_PID);
        old_PID_clear(&gimbal->gimbal_pos.Pitch_Vel_PID);

        return;
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_CALIBRATE){
        gimbal_calibrate_control(gimbal);
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_FOLLOW_CHASSIS){
        gimbal_calibrate_control(gimbal);
    }
    else if(gimbal->rc_control_mode->gimbal_mode == GIMBAL_FREE_CONTROL){

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

    if(gimbal->gimbal_shoot.pull_motor->target_current > 9000)
        gimbal->gimbal_shoot.pull_motor->target_current=9000;
    if(gimbal->gimbal_shoot.pull_motor->target_current < -9000)
        gimbal->gimbal_shoot.pull_motor->target_current=-9000;

    
}