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
#include "led.h"
// //#include "led.h"

// #include "UI_task.h"

control_mode_t control_mode;

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

//按键开关小陀螺
#define ROTATION_ON_KEYBOARD   KEY_PRESSED_OFFSET_SHIFT
//传入UI的小陀螺状态的全局变量
uint8_t roation_state=0;

static void chassis_zero_force_control();
static void chassis_no_move_control(float *vx_set, float *l_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);


//底盘行为状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //遥控器设置行为模式
    if (switch_is_down(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
    else if (switch_is_mid(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_up(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_mid(remote_ctrl->rc.s[FUNCTIONAL_CHANNEL])&&switch_is_down(control_mode.last_rc_func_state))
    {
        if (control_mode.rotation_state)
        {
            chassis_move.chassis_mode=CHASSIS_ROTATION_EXIT;
            control_mode.rotation_state=!control_mode.rotation_state;
        }
        else
        {
            chassis_move.chassis_mode=CHASSIS_ROTATION;
            control_mode.rotation_state=!control_mode.rotation_state;
        }
    }
    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move.chassis_mode = CHASSIS_FORCE_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_move.chassis_mode = CHASSIS_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_GIMBAL_YAW)
    {
        chassis_move.chassis_mode = CHASSIS_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }

}

TickType_t start_calibrate_time;

void gimbal_behaviour_mode_set(Remote_Info_Typedef const* remote_ctrl)
{
    if (remote_ctrl == NULL)
    {
        return;
    }
    control_mode.last_gimbal_behaviour_mode = control_mode.gimbal_behaviour_mode;

    //遥控器设置行为模式
    if (switch_is_up(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.gimbal_behaviour_mode = GIMBAL_FREE_CONTROL;
    }
    else if (switch_is_mid(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        if(control_mode.last_gimbal_behaviour_mode == GIMBAL_DISABLE)
        {
            control_mode.gimbal_behaviour_mode = GIMBAL_CALIBRATE;
            start_calibrate_time = xTaskGetTickCount();
        }
        else if(control_mode.last_gimbal_behaviour_mode == GIMBAL_CALIBRATE)
        {
            if( xTaskGetTickCount() - start_calibrate_time > 2000)
                control_mode.gimbal_behaviour_mode = GIMBAL_FOLLOW_CHASSIS;
        }
        else if(control_mode.last_gimbal_behaviour_mode == GIMBAL_FREE_CONTROL)
        {
            control_mode.gimbal_behaviour_mode = GIMBAL_FOLLOW_CHASSIS;
        }
    }
    else if (switch_is_down(remote_ctrl->rc.s[MODE_CHANNEL]))
    {
        control_mode.gimbal_behaviour_mode = GIMBAL_DISABLE;
        control_mode.self_turn_flag=false;
        control_mode.auto_shoot_flag = false;
        control_mode.shoot_flag = false;
        control_mode.shoot_fire_flag = false;
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
        control_mode.shoot_fire_flag =! control_mode.shoot_fire_flag;
    }
    else if(remote_ctrl->rc.ch[RC_ROLL_CH] < 400)
        control_mode.shoot_fire_flag=false;

    control_mode.last_rc_func_state = remote_ctrl->rc.s[FUNCTIONAL_CHANNEL];
    control_mode.last_shoot_channel = remote_ctrl->rc.ch[RC_ROLL_CH];
}

const control_mode_t* get_rc_control_mode(){
    return &control_mode;
}
void chassis_behaviour_control_set(float *vx_set, float *l_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || l_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
    // else if(chassis_behaviour_mode == CHASSIS_STAND_UP)
    // {
    //     chassis_stand_up_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    // }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
}
static void chassis_zero_force_control()
{
    ALLChassisTxFrame.Data[0] = 0 ;
    ALLChassisTxFrame.Data[1] = 0;
    ALLChassisTxFrame.Data[2] = 0 ;
    ALLChassisTxFrame.Data[3] = 0;
    ALLChassisTxFrame.Data[4] = 0 ;
    ALLChassisTxFrame.Data[5] = 0;
    ALLChassisTxFrame.Data[6] = 0 ;
    ALLChassisTxFrame.Data[7] = 0;
    USER_CAN_TxMessage(&ALLChassisTxFrame);
}
