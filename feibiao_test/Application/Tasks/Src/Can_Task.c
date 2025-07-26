#include "tim.h"
#include "main.h"
#include "can_task.h"
#include "Chassis_Task.h"
#include "old_pid.h"

//#define DEBUG
#define rc_deadline_limit(input, output, dealine)          \
{                                                      \
if ((input) > (dealine) || (input) < -(dealine)) { \
(output) = (input);                            \
} else {                                           \
(output) = 0;                                  \
}                                                  \
}

Feibiao_control_t feibiao;

static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left,int16_t current_Shoot_Right,int16_t current_Shoot_Pull);
static void Dji_Motor_Chassis_Can_Send(int16_t current_id_1,int16_t current_id_2,int16_t current_id_3,int16_t current_id_4);
static void Dji_Motor_Pitch_Can_Send(int16_t current_pitch);

static void FeiBiao_Send(int16_t yaw_6020, int16_t roll_6020, int16_t pull_3508);
extern CAN_TxFrameTypeDef FEIBIAO_MOTOR_FRAME;

void feibiao_init(Feibiao_control_t *feibiao);
void feibiao_feedback(Feibiao_control_t *feibiao);
// 设置控制量
void feibiao_setcontrol(Feibiao_control_t *feibiao);
void feibiao_control_loop(Feibiao_control_t *feibiao);



extern CAN_TxFrameTypeDef ALLShootTxFrame;
extern CAN_TxFrameTypeDef ALLChassisTxFrame;
extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;

void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
  osDelay(800);
	feibiao_init(&feibiao);

  osDelay(10);

  for(;;)
  {
  	// 仅作LED显示
  	if (remote_ctrl.rc.s[0]== 1)
  	{
  		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  	}
  	else if (remote_ctrl.rc.s[0] == 2)
  	{
  		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  	}
  	else if (remote_ctrl.rc.s[0] == 3)
  	{
  		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  	}
  	feibiao_feedback(&feibiao);
  	feibiao_setcontrol(&feibiao);
  	feibiao_control_loop(&feibiao);

  	FeiBiao_Send(feibiao.yaw_6020->target_current, feibiao.roll_6020->target_current, feibiao.pull_3508->target_current);


    osDelay(2);
  }
  /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
    float span = x_max-x_min;
    float offset =x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void feibiao_init(Feibiao_control_t *feibiao)
{

	feibiao->feibiao_rc = get_remote_control_point();

	feibiao->pull_3508 = get_feibiao_pull_motor();
	feibiao->yaw_6020  = get_feibiao_yaw_motor();
	feibiao->roll_6020 = get_feibiao_rollmotor();

	const static fp32 feibiao_yaw_pid[3]  = {YAW_6020_PID_KP, YAW_6020_PID_KI, YAW_6020_PID_KD};
	const static fp32 feibiao_roll_pid[3] = {ROLL_6020_PID_KP, ROLL_6020_PID_KI, ROLL_6020_PID_KD};
	const static fp32 feibiao_pull_pid[3] = {FEIBIAO_M3505_PID_KP, FEIBIAO_M3505_PID_KI, FEIBIAO_M3505_PID_KD};

	old_PID_Init(&feibiao->pull_3508_pid, PID_POSITION, feibiao_pull_pid, FEIBIAO_M3505_PID_MAX_OUT, FEIBIAO_M3505_PID_MAX_IOUT);
	old_PID_Init(&feibiao->yaw_6020_pid, PID_POSITION, feibiao_yaw_pid, YAW_6020_PID_MAX_OUT, YAW_6020_PID_MAX_IOUT);
	old_PID_Init(&feibiao->roll_6020_pid, PID_POSITION, feibiao_roll_pid, ROLL_6020_PID_MAX_OUT, ROLL_6020_PID_MAX_IOUT);

	//
	feibiao_feedback(feibiao);
	feibiao->next_yaw_pos = feibiao->yaw_6020->real_pos;
	feibiao->tmp_yaw      = feibiao->yaw_6020->real_pos;
}
void feibiao_feedback(Feibiao_control_t *feibiao)
{
	feibiao->roll_6020->real_pos = (((int)(feibiao->roll_6020->pos) - 5263) / 8192.0f) * 2.0f * 3.14159f; // rad
	feibiao->roll_6020->real_w   = feibiao->roll_6020->rpm * 2 * PI / 60.0f;
	if (feibiao->roll_6020->real_pos < -3.1415f) {
		feibiao->roll_6020->real_pos += 2.0f * 3.14159f;
	}

	feibiao->yaw_6020->real_pos = (((int)(feibiao->yaw_6020->pos) - 1139) / 8192.0f) * 2.0f * 3.14159f; // rad
	feibiao->yaw_6020->real_w   = feibiao->yaw_6020->rpm * 2 * PI / 60.0f;
	if (feibiao->yaw_6020->real_pos > 3.1415f) {
		feibiao->yaw_6020->real_pos -= 2.0f * 3.14159f;
	}

	feibiao->pull_3508->real_w = feibiao->pull_3508->rpm * 2 * PI / (60.0f * 19.0f);
}
void feibiao_setcontrol(Feibiao_control_t *feibiao)
{
	// if (switch_is_down(feibiao->feibiao_rc->rc.s[MODE_CHANNEL])) {
		// 清空控制。防止意外
		//    old_PID_clear(&feibiao->pull_3508_pid);
		//    old_PID_clear(&feibiao->yaw_6020_pid);
		//    old_PID_clear(&feibiao->roll_6020_pid);

		//    feibiao->is_pull = false;
		//    feibiao->target_w_3508 = 0.0f;
		// Set_PWM_Pulse(1000);
		//__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 4000);
		if (switch_is_up(feibiao->feibiao_rc->rc.s[MODE_CHANNEL]) )                                                       // 右拨杆向下、左拨杆下
		{ __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500); }
		if (switch_is_mid(feibiao->feibiao_rc->rc.s[MODE_CHANNEL]) )                                                       // 右拨杆向下、左拨杆下
		{ __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 1500); }                                                                                                                              // 舵机位置
		if (switch_is_down(feibiao->feibiao_rc->rc.s[MODE_CHANNEL]) )                                                       // 右拨杆向下、左拨杆下
		{ __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 2000); }                                                                                                                              // 舵机位置
		// if (switch_is_down(feibiao->feibiao_rc->rc.s[MODE_CHANNEL]) && switch_is_mid(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL])) { __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2500); } // 差30度
		// return;
	// } else if (switch_is_mid(feibiao->feibiao_rc->rc.s[MODE_CHANNEL])) {
	// 	feibiao->is_pull = false;
	// 	// 换飞镖的代码，切换下一个飞镖
	// 	// 左边拨到上面切换
	// 	if (switch_is_up(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL]) && feibiao->is_load_feibiao == false) {
	// 		feibiao->is_load_feibiao    = true;
	// 		feibiao->roll_6020_load_pos = feibiao->roll_6020->real_pos;
	// 	}
	//
	// } else if (switch_is_up(feibiao->feibiao_rc->rc.s[MODE_CHANNEL])) {
	// 	// 使用滑杆
	// 	// feibiao->target_w_3508 = 1.0f;
		feibiao->is_pull = true;
	// }
}
void feibiao_control_loop(Feibiao_control_t *feibiao)
{
    //  if( switch_is_down(feibiao->feibiao_rc->rc.s[MODE_CHANNEL]) ){
    //    //清空控制。防止意外
    //    old_PID_clear(&feibiao->pull_3508_pid);
    //    old_PID_clear(&feibiao->yaw_6020_pid);
    //    old_PID_clear(&feibiao->roll_6020_pid);

    //    feibiao->roll_6020->target_current = 0;
    //    feibiao->pull_3508->target_current = 0;
    //    feibiao->yaw_6020->target_current = 0;
    //    return;
    //  }
    //////////////////////////
    // 飞镖装填
    ////////////////////////////
    fp32 tmp_target = 0.0f;
    if (switch_is_mid(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL])) {
        if (feibiao->is_load_feibiao == true) {
            fp32 next_load_pos  = feibiao->roll_6020_load_pos + PI * 2.0f / 3.0f;
            feibiao->target_pos = rad_format(next_load_pos);
            tmp_target          = feibiao->target_pos;

            if (feibiao->target_pos - feibiao->roll_6020->real_pos < -PI * 1.0f / 3.0f) {
                feibiao->target_pos = PI - feibiao->target_pos;
            } else if (feibiao->target_pos - feibiao->roll_6020->real_pos > PI) {
                feibiao->target_pos = tmp_target;
            }
            // 计算PID
            old_PID_Calc(&feibiao->roll_6020_pid, feibiao->roll_6020->real_pos, feibiao->target_pos);
            feibiao->roll_6020->target_current = (int16_t)feibiao->roll_6020_pid.out;
        }
    }

    if (switch_is_down(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL])) {
        feibiao->is_load_feibiao = false;
    }
    //////////////////////////
    // 滑台下降
    /////////////////////////
    if (feibiao->is_pull == true) {
        if (switch_is_up(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL])) {
            feibiao->target_w_3508 = TARGET_SPD;
            old_PID_Calc(&feibiao->pull_3508_pid, feibiao->pull_3508->real_w, feibiao->target_w_3508);
            feibiao->pull_3508->target_current = feibiao->pull_3508_pid.out;
        } else if (switch_is_down(feibiao->feibiao_rc->rc.s[FUNCTIONAL_CHANNEL])) {
            feibiao->target_w_3508 = -TARGET_SPD;
            old_PID_Calc(&feibiao->pull_3508_pid, feibiao->pull_3508->real_w, feibiao->target_w_3508);
            feibiao->pull_3508->target_current = feibiao->pull_3508_pid.out;
        } else
            feibiao->is_pull = false;
    }

    if (feibiao->is_pull == false) {
        old_PID_clear(&feibiao->pull_3508_pid);
        feibiao->pull_3508->target_current = 0;
    }
    /////////////////////////////////////
    // YAW控制
    ////////////////////////////////////
    int16_t add_yaw_channel;
    rc_deadline_limit(feibiao->feibiao_rc->rc.ch[RC_RIGHT_X_CH], add_yaw_channel, 20);
    fp32 add_yaw = add_yaw_channel * RC_TO_ADDYAW;

    feibiao->tmp_yaw = rad_format(feibiao->tmp_yaw + add_yaw);
    if (fabs(feibiao->tmp_yaw - feibiao->yaw_6020->real_pos) > PI) {
        if (feibiao->tmp_yaw > 0.0f) {
            feibiao->next_yaw_pos = -2.0f * PI + feibiao->tmp_yaw;
        } else if (feibiao->tmp_yaw < 0.0f) {
            feibiao->next_yaw_pos = 2.0f * PI + feibiao->tmp_yaw;
        }
    } else {
        feibiao->next_yaw_pos = feibiao->tmp_yaw;
    }

    old_PID_Calc(&feibiao->yaw_6020_pid, feibiao->yaw_6020->real_pos, feibiao->next_yaw_pos);
    feibiao->yaw_6020->target_current = feibiao->yaw_6020_pid.out;

    ///////////////////////////////
    // 舵机控制
    ///////////////////////////////

    //??
}
static void FeiBiao_Send(int16_t yaw_6020, int16_t roll_6020, int16_t pull_3508)
{
    FEIBIAO_MOTOR_FRAME.Data[0] = yaw_6020 >> 8;
    FEIBIAO_MOTOR_FRAME.Data[1] = yaw_6020;
    FEIBIAO_MOTOR_FRAME.Data[2] = roll_6020 >> 8;
    FEIBIAO_MOTOR_FRAME.Data[3] = roll_6020;
    FEIBIAO_MOTOR_FRAME.Data[4] = pull_3508 >> 8;
    FEIBIAO_MOTOR_FRAME.Data[5] = pull_3508;
    FEIBIAO_MOTOR_FRAME.Data[6] = 0;
    FEIBIAO_MOTOR_FRAME.Data[7] = 0;

    USER_CAN_TxMessage(&FEIBIAO_MOTOR_FRAME);
}


