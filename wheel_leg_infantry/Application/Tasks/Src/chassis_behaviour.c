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
#include "chassis_task.h"

#include "arm_math.h"
#include "User_Task.h"
//#include "led.h"
// gimbal_chassis_comm_t * local_gimbal_chassis_comm ;

#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_can_set 前进的速度 设定值将直接发送到can总线上
  * @param[in]      l_can_set  腿长的设定值 设定值将直接发送到can总线上
  * @param[in]      angle_can_set 角度的设定值 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(float *vx_can_set, float *l_can_set, float *angle_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘站立的行为状态机下，底盘调整状态完成站立操作
  * @author         RM
  * @param[in]      vx_set 前进的速度 设定值将直接发送到can总线上
  * @param[in]      l_set 腿长的设定值 设定值将直接发送到can总线上
  * @param[in]      angle_set 角度的设定值 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_stand_up_control(float *vx_set, float *l_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set 前进的速度
  * @param[in]      l_set 腿长的设定值
  * @param[in]      angle_set 角度的设定值
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(float *vx_set, float *l_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随云台角度的行为状态机下，底盘模式是跟随角度，底盘旋转速度由云台参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
// 新增：分步站立控制函数
static void chassis_get_up_auto(fp32 *l_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);


//底盘行为状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
#ifdef CAN1_Chassis_RC_Mod
    if (gimbal_chassis_comm.comm_ok)
    {
        chassis_behaviour_mode=gimbal_chassis_comm.gimbal_cmd.chassis_mode_cmd;
        chassis_move_mode.spining_flag=gimbal_chassis_comm.gimbal_cmd.spinning_cmd;
        chassis_move_mode->jump_state.jump_flag=gimbal_chassis_comm.gimbal_cmd.jump_cmd;
        chassis_move_mode->vx_from_ros=gimbal_chassis_comm.gimbal_cmd.target_speed_x;
        chassis_move_mode->wz_from_ros=gimbal_chassis_comm.gimbal_cmd.target_speed_w_z;
        chassis_move_mode->leg_length_set=gimbal_chassis_comm.gimbal_cmd.target_length;
        // chassis_move_mode->chassis_yaw_set=gimbal_chassis_comm.gimbal_cmd.gimbal_yaw_angle;
    }
    //  if (chassis_move_mode->chassis_can_rc_info->s[LEFT_1_SWITCH]==-1)
    // {
    //     chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    // }
    // else if (chassis_move_mode->chassis_can_rc_info->s[LEFT_1_SWITCH]==1)
    // {
    //     chassis_behaviour_mode = CHASSIS_NO_MOVE;
    // }
    //  if ((chassis_move_mode->chassis_can_rc_info->s[LEFT_2_SWITCH]==-1)&&(chassis_move_mode->chassis_can_rc_info->s[LEFT_1_SWITCH]==1))
    // {
    //     chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    // }
    // else if ((chassis_move_mode->chassis_can_rc_info->s[LEFT_2_SWITCH]==1)&&(chassis_move_mode->chassis_can_rc_info->s[LEFT_1_SWITCH]==1))
    //  {
    //      chassis_behaviour_mode = CHASSIS_FOLLOW_YAW;
    //  }
    // chassis_move_mode->last_chassis_funtion_1_mode=chassis_move_mode->chassis_can_rc_info->s[RIGHT_1_SWITCH];
    // chassis_move_mode->last_chassis_funtion_2_mode=chassis_move_mode->chassis_can_rc_info->s[RIGHT_2_SWITCH];

#else
    //遥控器设置行为模式
    if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_FOLLOW_YAW;
    }
    else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
    }
    chassis_move_mode->last_chassis_funtion_mode=chassis_move_mode->chassis_RC->rc.s[FUNTION_CHANNEL];
#endif

    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_FORCE_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
    else if(chassis_behaviour_mode == CHASSIS_STAND_UP)
    {
        chassis_move_mode->chassis_mode = CHASSIS_FORCE_RAW; //当行为是站立时，则设置底盘状态机为 raw，原生状态机。
    }
    // else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    // {
    //     // chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    // }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        // chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //当行为是底盘跟随角度，则设置底盘状态机为 底盘跟随角度 状态机。
    }
}

// TODO：腿长控制在状态机切换时需要保存，这里没有办法保存
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
    //TODO: 添加机器人站立控制逻辑，若是上电第一次站立，还需要校准关节电机角度
    else if(chassis_behaviour_mode == CHASSIS_STAND_UP)
    {
        chassis_stand_up_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
    // else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    // {
    //     chassis_no_move_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    // }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_YAW)
    {
        chassis_follow_yaw_control(vx_set, l_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_can_set 前进的速度 设定值将直接发送到can总线上
  * @param[in]      l_can_set  腿长的设定值 设定值将直接发送到can总线上
  * @param[in]      wz_can_set 角度的设定值 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_zero_force_control(float *vx_can_set, float *l_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || l_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *l_can_set = 0.0f;
    *wz_can_set = 0.0f;
}


/**
  * @brief          底盘站立的行为状态机下，底盘调整状态完成站立操作
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_stand_up_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set 前进的速度
  * @param[in]      l_set 腿长的设定值
  * @param[in]      angle_set 角度的设定值
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(float *vx_set, float *l_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || l_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    // *vx_set = 0.0f;
    // *l_set = LEG_LENGTH_INIT;
    // *angle_set = 0.0f;
    // 如果机器人还处于倾斜状态，则执行自动站立程序
    if (fabs(chassis_move_rc_to_vector->chassis_pitch) > 0.1f) // 0.1 rad ≈ 5.7 degrees
    {
        chassis_get_up_auto(l_set, angle_set, chassis_move_rc_to_vector);
        *vx_set = 0.0f; // 站立过程中不允许移动
    }
    else // 已经站稳，保持不动
    {
        *vx_set = 0.0f;
        *l_set = LEG_LENGTH_INIT;
        *angle_set = 0.0f;
    }
}

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
/**
  * @brief          底盘跟随角度的行为状态机下，底盘模式是跟随角度，底盘旋转速度由云台参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */

static void chassis_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}
/**
  * @brief          根据当前俯仰角自动分步站立
  * @author         GitHub Copilot
  * @param[in]      l_set 腿长设定值指针
  * @param[in]      angle_set 姿态角设定值指针
  * @param[in]      chassis_move_rc_to_vector 底盘数据
  * @retval         返回空
  */
// static void chassis_get_up_auto(fp32 *l_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
// {
//     fp32 pitch = chassis_move_rc_to_vector->chassis_pitch;
//
//     // 阶段1：严重倾斜（前倾或后仰超过 0.5rad）
//     if (fabsf(pitch) > 0.5f)
//     {
//         *l_set = LEG_LENGTH_MIN;  // 收腿降低重心
//
//         // **关键：根据倾斜方向给出反向角度补偿**
//         if (pitch < 0)
//             *angle_set = -pitch * 0.8f;   // 前倾 -> 反向抬头
//         else
//             *angle_set = -pitch * 0.8f;   // 后仰 -> 反向低头
//     }
//     // 阶段2：中度倾斜
//     else
//     {
//         // 平滑恢复
//         fp32 ratio = (fabsf(pitch) - 0.1f) / (0.5f - 0.1f);
//         ratio = fp32_constrain(ratio, 0.0f, 1.0f);
//
//         *l_set = LEG_LENGTH_INIT - (LEG_LENGTH_INIT - LEG_LENGTH_MIN) * ratio;
//
//         // 角度向水平平滑过渡（同样取反）
//         *angle_set = -pitch * 0.5f;
//     }
//
//     // 限制范围
//     *l_set = fp32_constrain(*l_set, LEG_LENGTH_MIN, LEG_LENGTH_INIT);
// }
/**
  * @brief          并联腿机器人惯性起立函数（弧度制，自动区分前倾/后仰）
  * @author         ChatGPT
  * @param[in,out]  l_set 腿长设定值指针
  * @param[in,out]  angle_set 姿态角设定值指针
  * @param[in]      chassis_move_rc_to_vector 底盘状态数据
  * @retval         无
  */
/**
  * @brief   强力惯性起立函数（弧度制、自动区分前倾/后仰）
  * @author  ChatGPT
  */
static void chassis_get_up_auto(fp32 *l_set, fp32 *angle_set, chassis_move_t *chassis)
{
    fp32 pitch = chassis->chassis_pitch; // 弧度制
    static uint32_t last_tick = 0;
    uint32_t now_tick = xTaskGetTickCount();
    uint32_t t = now_tick - last_tick;
    last_tick = now_tick;

    // 默认设为安全值
    *l_set = LEG_LENGTH_INIT;
    *angle_set = 0.0f;

    // --------------------
    // 1. 倒地蓄力阶段
    // --------------------
    if (fabsf(pitch) > 0.9f)
    {
        *l_set = LEG_LENGTH_MIN; // 收腿
        // 前倾收腿抬头、后仰收腿低头
        *angle_set = (pitch < 0) ? +1.0f : -1.0f;
        return;
    }

    // --------------------
    // 2. 惯性甩起阶段
    // --------------------
    if (fabsf(pitch) > 0.35f && fabsf(pitch) <= 0.9f)
    {
        // 快速伸腿，同时反向甩角
        float phase = sinf((float)t * 0.02f);
        *l_set = LEG_LENGTH_MIN + 0.05f * phase; // 动态腿长扰动

        if (pitch < 0)      // 前倾：需要向后打力矩（反时针）
            *angle_set = +1.3f;
        else                // 后仰：需要向前打力矩（顺时针）
            *angle_set = -1.3f;

        // 可选：足端轮反转推力（增强起立）
        chassis->left_leg.wheel_motor.give_current  = -1000.0f;
        chassis->right_leg.wheel_motor.give_current = -1000.0f;
        return;
    }

    // --------------------
    // 3. 伸腿扶起阶段
    // --------------------
    if (fabsf(pitch) > 0.15f && fabsf(pitch) <= 0.35f)
    {
        float k = (fabsf(pitch) - 0.15f) / (0.35f - 0.15f);
        *l_set = LEG_LENGTH_MIN + (LEG_LENGTH_INIT - LEG_LENGTH_MIN) * (1.0f - k);
        *angle_set = -pitch * 0.8f;
        return;
    }

    // --------------------
    // 4. 稳定平衡阶段
    // --------------------
    *l_set = LEG_LENGTH_INIT;
    *angle_set = -pitch * 0.4f;
}

