#include "Chassis_task.h"
#include "main.h"
#include "remote_control.h"

#include "chassis_behaviour.h"
#include "usb.h"
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


//底盘运动数据
 chassis_move_t chassis_move;
// 底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
// //底盘初始化，主要是pid初始化
// static void chassis_init(chassis_move_t *chassis_move_init);
// //设置遥控器状态
// void chassis_set_mode(chassis_move_t *chassis_move_mode);
// //切换模式的时候数据处理部分
// void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//设置控制量
void chassis_set_contorl(chassis_move_t *chassis_move_control);
// //loop
void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  osDelay(100);
  TickType_t systick = 0;

  chassis_init(&chassis_move);
  for(;;)
  {

      // HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

    systick = osKernelSysTick();

    chassis_behaviour_mode_set(chassis_move.chassis_RC);
    // //遥控器状态切换数据保存
    // chassis_mode_change_control_transit(&chassis_move);
    //底盘数据更新
    chassis_feedback_update(&chassis_move);
    // //底盘控制量设置
    chassis_set_contorl(&chassis_move);
    // //底盘控制PID计算
    chassis_control_loop(&chassis_move);
    //printf("%.2f,%.2f\r\n",chassis_move.right_leg.leg_length,chassis_move.left_leg.leg_length);
    //西普开源
    //chassis_power_control(&chassis_move);

    osDelayUntil(&systick,2);
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

    chassis_move_init->chassis_RC = get_remote_control_point();
    chassis_move_init->rc_control_mode = get_rc_control_mode();

    chassis_move_init->chassis_motor[0] = get_chassis_motor(0);
    chassis_move_init->chassis_motor[1] = get_chassis_motor(1);
    chassis_move_init->chassis_motor[2] = get_chassis_motor(2);
    chassis_move_init->chassis_motor[3] = get_chassis_motor(3);

    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static fp32 state_xdot_constant[1] = {0};
    //ROS_PID
    const static fp32 chassis_ros_pid[3]={ROS_WZ_PID_KP,ROS_WZ_PID_KI,ROS_WZ_PID_KD};

    chassis_move_init->kilometer = 0.0f;
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //初始化PID 运动
    for (uint8_t i = 0; i < 4; i++)
    {
        old_PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //ros
    old_PID_Init(&chassis_move_init->chassis_ros_wz_pid,PID_POSITION,chassis_ros_pid,ROS_WZ_PID_MAX_OUT,ROS_WZ_PID_MAX_IOUT);

    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
    first_order_filter_init(&chassis_move_init->state_xdot_filter, CHASSIS_CONTROL_TIME, state_xdot_constant);

    //小陀螺旋转 斜波函数缓启
    ramp_init(&chassis_move_init->rotation_ramp_wz, CHASSIS_CONTROL_TIME, ROTATION_SPEED_MAX, 0);

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}
static void chassis_feedback_update(chassis_move_t *chassis_move_update){
    if (chassis_move_update == NULL)
    {
        return;
    }
    extern Usb_dpkg_data_t* Usb_receive_data;
    for (uint8_t i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->chassis_motor[i]->accel = (chassis_move_update->chassis_motor[i]->real_w * WHEELR - chassis_move_update->chassis_motor[i]->speed)/CHASSIS_CONTROL_TIME;
        chassis_move_update->chassis_motor[i]->speed = chassis_move_update->chassis_motor[i]->real_w * WHEELR;
    }

    chassis_move_update->vx_from_ros = Usb_receive_data->vx_set;
    chassis_move_update->wz_from_ros = Usb_receive_data->wz_set;
        //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vel_ref.vx = (-chassis_move_update->chassis_motor[0]->speed + chassis_move_update->chassis_motor[1]->speed + chassis_move_update->chassis_motor[2]->speed - chassis_move_update->chassis_motor[3]->speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vel_ref.vy = (-chassis_move_update->chassis_motor[0]->speed - chassis_move_update->chassis_motor[1]->speed + chassis_move_update->chassis_motor[2]->speed + chassis_move_update->chassis_motor[3]->speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->vel_ref.wz= (-chassis_move_update->chassis_motor[0]->speed - chassis_move_update->chassis_motor[1]->speed - chassis_move_update->chassis_motor[2]->speed - chassis_move_update->chassis_motor[3]->speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    first_order_filter_cali(&chassis_move_update->state_xdot_filter, chassis_move_update->vel_ref.vx);

    chassis_move_update->kilometer +=  sqrt((pow(chassis_move_update->vel_ref.vx,2)+pow(chassis_move_update->vel_ref.vy,2))* CHASSIS_CONTROL_TIME);

    //chassis_move.chassis_yaw = 


}
void fp_limit_value(float* val,float MAX,float MIN){
    if(*val  > MAX){
        *val = MAX;
    }
    else if(*val < MIN){
        *val = MIN;
    }
}
/**
  * @brief          遥控器的数据处理成底盘的前进vx速度，vy速度
  * @author         pxx
  * @param          vx_set  x轴前进速度设置，m/s
  * @param          vy_set  y轴前进速度设置，m/s
  * @param          chassis_move_rc_to_vector   底盘结构体指针
  * @retval         void
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set,fp32 *angle_set ,chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL || angle_set==NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel,wz_channel;
    fp32 vx_set_channel, vy_set_channel,wz_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL], wz_channel, CHASSIS_RC_DEADLINE);

    // vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
    // wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;

    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    // {
    //     vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    // }
    //
    // if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    // {
    //     vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    // }
    // else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    // {
    //     vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    // }
    ///////////////////////ROS////////////////////////////////////////////
    if(wz_channel != 0)
        wz_set_channel = wz_channel * CHASSIS_WZ_RC_SEN;
    else
        wz_set_channel = chassis_move_rc_to_vector->wz_from_ros * CHASSIS_ROS_TO_WZ;

    if(vx_channel != 0)
        vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    else
        vx_set_channel = chassis_move_rc_to_vector->vx_from_ros;
    ///////////////////////////////////////ROS/////////////////////
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;

    fp_limit_value(vx_set,NORMAL_MAX_CHASSIS_SPEED_X,-NORMAL_MAX_CHASSIS_SPEED_X);
    fp_limit_value(vy_set,NORMAL_MAX_CHASSIS_SPEED_Y,-NORMAL_MAX_CHASSIS_SPEED_Y);

    *angle_set = wz_set_channel;
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

    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_rc_to_control_vector(&vx_set,&vy_set,&angle_set,chassis_move_control);
    //遥控输入给底盘，云台自动跟随保持固定角度
    if(chassis_move_control->rc_control_mode->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW){
        
    }
    else if(chassis_move_control->rc_control_mode->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW){
        //wan dian zai xie;
        //底盘跟随云台
        //chassis_rc_to_control_vector(&vx_set,&vy_set,&angle_set,chassis_move_control);
        //x,y依然使用遥控，wz需要使用PID计算
    }
    else if(chassis_move_control->rc_control_mode->chassis_mode == CHASSIS_FORCE_RAW){
        vx_set = 0.0f;
        vy_set = 0.0f;
        angle_set = 0.0f;
    }

    chassis_move_control->vel_set.vx = vx_set;
    chassis_move_control->vel_set.vy = vy_set;
    chassis_move_control->vel_set.wz = angle_set;
}

/**
  * @brief          麦轮运动分解
  * @author         pxx
  * @param          vx_set  x轴前进速度设置，m/s
  * @param          vy_set  y轴前进速度设置，m/s
  * @param          wz_set  z轴旋转速度设置，rad/s
  * @param          wheel_speed 四个轮子的转速
  * @retval         void
  */
void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

void chassis_control_loop(chassis_move_t *chassis_move_control_loop){

    if(chassis_move_control_loop->rc_control_mode->chassis_mode == CHASSIS_FORCE_RAW){
            //赋值电流值
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_control_loop->chassis_motor[i]->target_current = 0.0f;
        }

        for (uint8_t i = 0; i < 4; i++)
        {
            old_PID_clear(&chassis_move_control_loop->motor_speed_pid[i]);
        }

        return;
    }

    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vel_set.vx,\
                                        chassis_move_control_loop->vel_set.vy,\
                                        chassis_move_control_loop->vel_set.wz,\
                                        wheel_speed);

    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    //计算轮子控制最大速度，并限制其最大速度
    for (uint8_t i = 0; i < 4; i++)
    {
        temp = fabs(wheel_speed[i]);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (uint8_t i = 0; i < 4; i++)
        {
            wheel_speed[i] *= vector_rate;
        }
    }

    //计算pid
    for (uint8_t i = 0; i < 4; i++)
    {
        old_PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->chassis_motor[i]->speed, wheel_speed[i]);
    }
    chassis_move_control_loop->tmp = wheel_speed[1];
    //chassis_power_control(chassis_move_control_loop);

    //赋值电流值
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control_loop->chassis_motor[i]->target_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
        
        // if(wheel_speed[i] > 0.05f)
        //     chassis_move_control_loop->chassis_motor[i]->target_current+=1200.0f;
        // else if(wheel_speed[i] < -0.05f)
        //     chassis_move_control_loop->chassis_motor[i]->target_current-=1200.0f;
    }

    // chassis_move_control_loop->chassis_motor[0]->target_current = 1400;
    // chassis_move_control_loop->chassis_motor[1]->target_current = -1400;
    // chassis_move_control_loop->chassis_motor[2]->target_current = -1400;
    // chassis_move_control_loop->chassis_motor[3]->target_current = 1400;
}

const chassis_move_t* get_chassis_point(){
  //底盘运动数据
  return &chassis_move;
}