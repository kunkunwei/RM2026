/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  v2.0.0     Nov-05-2023     pxx             刚起步
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"
#include "remote_control.h"
#include "chassis_task.h"

#include "arm_math.h"
// //#include "led.h"

control_mode_t control_mode;

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)


void chassis_behaviour_mode_set(Remote_Info_Typedef const* remote_ctrl)
{
    if (remote_ctrl == NULL)
    {
        return;
    }
    control_mode.last_chassis_mode = control_mode.chassis_mode;

    //遥控器设置行为模式
    if (switch_is_up(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //底盘跟随云台
    }
    else if (switch_is_mid(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;//云台和车身固定朝向
    }
    else if (switch_is_down(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.chassis_mode = CHASSIS_FORCE_RAW;//不做控制
        control_mode.self_turn_flag = false;
        return;
    }
    
    if(switch_is_mid(control_mode.last_rc_func_state) && switch_is_down(remote_ctrl->rc.s[FUNCTIONAL_CHANNEL]))
    {
        control_mode.self_turn_flag = ~control_mode.self_turn_flag;
    }

    
    control_mode.last_rc_func_state = remote_ctrl->rc.s[FUNCTIONAL_CHANNEL];
}

TickType_t start_calibrate_time;

void gimbal_behaviour_mode_set(Remote_Info_Typedef const* remote_ctrl)
{
    if (remote_ctrl == NULL)
    {
        return;
    }
    control_mode.last_gimbal_mode = control_mode.gimbal_mode;

    //遥控器设置行为模式
    if (switch_is_up(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.gimbal_mode = GIMBAL_FREE_CONTROL;
    }
    else if (switch_is_mid(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        if(control_mode.last_gimbal_mode == GIMBAL_DISABLE)
        {
            control_mode.gimbal_mode = GIMBAL_CALIBRATE;
            start_calibrate_time = xTaskGetTickCount();
        }
        else if(control_mode.last_gimbal_mode == GIMBAL_CALIBRATE)
        {
            if( xTaskGetTickCount() - start_calibrate_time > 2000)
                control_mode.gimbal_mode = GIMBAL_FOLLOW_CHASSIS;
        }
        else if(control_mode.last_gimbal_mode == GIMBAL_FREE_CONTROL)
        {
            control_mode.gimbal_mode = GIMBAL_FOLLOW_CHASSIS;
        }
    }
    else if (switch_is_down(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.gimbal_mode = GIMBAL_DISABLE;
        control_mode.self_turn_flag=false;
        control_mode.auto_shoot_flag = false;
        control_mode.shoot_flag = false;
        control_mode.fire_flag = false;
        return ;
    }
    ///////左边拨杆,自瞄切换
    if(switch_is_mid(control_mode.last_rc_func_state) && switch_is_up(remote_ctrl->rc.s[FUNCTIONAL_CHANNEL]))
    {
        control_mode.auto_shoot_flag =  !control_mode.auto_shoot_flag;
    }
    //左边的小轮子，摩擦轮开转
    if(control_mode.last_shoot_channel > -400 && remote_ctrl->rc.ch[RC_ROLL_CH] < -400){
        control_mode.shoot_flag =! control_mode.shoot_flag;
    }
    //左边的小轮子，开火
    if(control_mode.last_shoot_channel < 400 && remote_ctrl->rc.ch[RC_ROLL_CH] > 400 && control_mode.shoot_flag==true){
        control_mode.fire_flag =! control_mode.fire_flag;
    }
    else if(remote_ctrl->rc.ch[RC_ROLL_CH] < 400)
        control_mode.fire_flag=false;

    control_mode.last_rc_func_state = remote_ctrl->rc.s[FUNCTIONAL_CHANNEL];
    control_mode.last_rc_mode_state = remote_ctrl->rc.s[0];
    control_mode.last_shoot_channel = remote_ctrl->rc.ch[RC_ROLL_CH];
}

const control_mode_t* get_rc_control_mode(){
    return &control_mode;
}