#include "observe_task.h"
#include "chassis_task.h"
#include "leg_angular_predictor.h"
#include "slip_detector.h"
 SlipDetector_t slip_detector;

///////////////////////////////////////////////////////////////
/*------------------机体速度与加速度估计KF--------------------------*/
KalmanFilter_Info_TypeDef vaEstimateKF;
float vaEstimateKF_F[4] = {1.0f, 0.005f,
                           0.0f, 1.0f};

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};

float vaEstimateKF_Q[4] = {10.0f, 0.0f,
                           0.0f, 15.0f};

float vaEstimateKF_R[4] = {100.0f, 0.0f,
                            0.0f,  300.0f};

// float vaEstimateKF_K[4];
//u(t)=[轮速,机体速度]^T
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};
///////////////////////////////////////////////////////////////
/*------------------各腿theta和theta微分估计KF--------------------------*/
LegPredictor_t leg_predictor;


///////////////////////////////////////////////////////////////

// 状态变量
static float vel_acc[2];
static fp32 v_real;
static fp32 aver_v;
static fp32 diff_v;
static fp32 body_v;
static const chassis_move_t* local_chassis_move;
extern const fp32 *local_imu_gyro;	  //imu数据
// 静态偏置估计：使用低通滤波估计IMU零偏
static fp32 imu_bias_estimate = -0.4f;
//extern bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);
/*-------------------------------------------------------------*/
static void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF);
static void CalculateWheelSpeed(const chassis_move_t *chassis, float *wr, float *wl, float *vrb, float *vlb);
static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v);
void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel);
/*-------------------------------------------------------------*/

/*-------------------------------------------------------------*/

void ObserveTask(void const * argument)
{
	// 初始化
    local_chassis_move = get_chassis_control_point();
	SlipDetector_Init(&slip_detector);
	xvEstimateKF_Init(&vaEstimateKF);
	LegPredictor_Init(&leg_predictor, local_chassis_move, 20.0f);
	// 局部变量
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
    static float last_v=0.0f;
	fp32 raw_imu_accel=0.0f;
	fp32 compensated_accel=0.0f;
  TickType_t systick = 0;
  while(1)
	{  
		osDelayUntil(&systick,5);
  		// 1. 打滑检测数据同步与更新
  		SlipDetector_SyncData(&slip_detector,
			local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f + local_chassis_move->left_leg.angle_dot,
			-local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f + local_chassis_move->right_leg.angle_dot,
			compensated_accel,
			local_chassis_move->left_leg.wheel_motor.give_current,
			local_chassis_move->right_leg.wheel_motor.give_current,
			local_chassis_move->wz_set);;
		SlipDetector_Update(&slip_detector);
  		// SlipDetector_GetConfidence(&slip_detector,&slip_detector.left.confidence,&slip_detector.right.confidence);

  		// 2. 计算轮速和车体速度
  		CalculateWheelSpeed(local_chassis_move, &wr, &wl, &vrb, &vlb);
  	    // 3. 计算平均速度和速度差
  	    last_v = aver_v;
  	    aver_v = (vrb + vlb) / 2.0f;
  	    diff_v = (wr - wl) * WHEEL_R - local_chassis_move->wz_set * 2 * MOTOR_DISTANCE_TO_CENTER;
  		if(diff_v >2.0f || diff_v <-2.0f){
  			//滑了
  			// 调整矩阵R
  			vaEstimateKF.Data.R[0] = 5000.0f;
  		}
  		else{
  			vaEstimateKF.Data.R[0] = 250.0f;
  		}
  		// 5. 更新IMU偏置
  		raw_imu_accel = *(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET);
  		UpdateIMUBias(raw_imu_accel, aver_v);


  	// 使用自适应偏置补偿后的加速度
  		compensated_accel = raw_imu_accel + imu_bias_estimate;

  	xvEstimateKF_Update(&vaEstimateKF, compensated_accel, aver_v);

    body_v = vel_acc[0];
    v_real = vel_acc[0] \
                -local_chassis_move->leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta);

  	 LegPredictor_Update(&leg_predictor,local_chassis_move,local_chassis_move->leg_tor,
	    local_chassis_move->wheel_tor,local_chassis_move->err_tor);
                -local_chassis_move->leg_length*arm_sin_f32(local_chassis_move->state_ref.theta);//
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
/*-------------------------------机体速度与加速度估计KF-------------------------------------------*/
/**
 * @brief 初始化卡尔曼滤波器
 */
static void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 观测维度
	
	memcpy(EstimateKF->Data.A, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->Data.P, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Data.Q, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->Data.R, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->Data.H, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}
/**
 * @brief 计算轮速和车体速度
 * @param chassis 底盘结构体指针
 * @param wr 右轮角速度输出
 * @param wl 左轮角速度输出
 * @param vrb 右轮线速度输出
 * @param vlb 左轮线速度输出
 */
static void CalculateWheelSpeed(const chassis_move_t *chassis, float *wr, float *wl, float *vrb, float *vlb)
{
	if (chassis == NULL || wr == NULL || wl == NULL || vrb == NULL || vlb == NULL) return;

	// 计算轮角速度
	*wr = -chassis->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f + chassis->right_leg.angle_dot;
	*wl =  chassis->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f + chassis->left_leg.angle_dot;

	// 计算轮线速度
	*vrb = *wr * WHEEL_R
		  + chassis->right_leg.leg_length * chassis->state_ref.theta_dot * arm_cos_f32(chassis->state_ref.theta)
		  + chassis->right_leg.length_dot * arm_sin_f32(chassis->state_ref.theta);

	*vlb = *wl * WHEEL_R
		  + chassis->left_leg.leg_length * chassis->state_ref.theta_dot * arm_cos_f32(chassis->state_ref.theta)
		  + chassis->left_leg.length_dot * arm_sin_f32(chassis->state_ref.theta);
}
/**
 * @brief 更新IMU加速度偏置（低通滤波）
 * @param raw_imu_accel 原始IMU加速度
 * @param aver_v 平均速度
 */
static void UpdateIMUBias(fp32 raw_imu_accel, float aver_v)
{
	// 静止状态下更新偏置（速度和加速度都很小）
	if (fabsf(aver_v) < 0.01f && fabsf(raw_imu_accel + imu_bias_estimate) < 0.5f) {
		imu_bias_estimate = imu_bias_estimate * (1.0f - 0.001f) + (-raw_imu_accel) * 0.001f;
	}
}

void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel)
{   	
    
    EstimateKF->MeasuredVector[0] =	vel;
    EstimateKF->MeasuredVector[1] = acc;
    		
    Kalman_Filter_Update(EstimateKF);

    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->Output[i];
    }
}
////////////////////////////////////////////////////////////////////////////////////////
// 基于简化WBR模型的腿摆杆角速度预测函数
// /**
//  * @brief 重置滤波器（使用当前实际状态初始化）
//  */
// void LegPredictor_Reset(chassis_move_t *chassis)
// {
// 	if ( chassis == NULL) return;
//
// 	// 获取当前实际状态
// 	float theta_left = chassis->left_leg.leg_angle - PI/2 - chassis->chassis_pitch;
// 	float theta_right = chassis->right_leg.leg_angle - PI/2 - chassis->chassis_pitch;
//
// 	float omega_left = chassis->left_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET);
// 	float omega_right = chassis->right_leg.angle_dot - *(chassis->chassis_imu_gyro + INS_GYRO_X_ADDRESS_OFFSET);
// 	//
// 	// // 初始化左腿KF
// 	// SingleLegKF_Init(&predictor->left_leg,
// 	// 				predictor->dt,
// 	// 				chassis->left_leg.leg_length,
// 	// 				2.8f,  // 默认等效质量，需要辨识
// 	// 				theta_left,
// 	// 				omega_left);
// 	//
// 	// // 初始化右腿KF
// 	// SingleLegKF_Init(&predictor->right_leg,
// 	// 				predictor->dt,
// 	// 				chassis->right_leg.leg_length,
// 	// 				2.8f,
// 	// 				theta_right,
// 	// 				omega_right);
// }

////////////////////////////////////////////////////////////////////////////////////////
fp32 get_KF_Spd(void)
{   	
    return v_real;
}
fp32 get_raw_Spd(void)
{   	
    return aver_v;
}
fp32 get_diff_Spd(void)
{   	
    return diff_v;
}
fp32 get_body_Spd(void)
{   	
    return body_v;
}
fp32 get_imu_bias(void)
{
	return  imu_bias_estimate;
}
SlipDetector_t* get_slip_detector_point(void)
{
	return &slip_detector;
}
 SlipFlag get_slip_flag(void)
{
	return slip_detector.slip_flag;
}
fp32 get_confidence_left(void)
{
	return slip_detector.left.confidence;
}
fp32 get_confidence_right(void)
{
	return slip_detector.right.confidence;
}
const LegPredictor_t *get_leg_predictor_point(void)
{
	return &leg_predictor;
}
/**
 * @brief 获取补偿力矩（供控制循环使用）
 * @param predictor 预测器指针
 * @param comp_left 输出：左腿补偿力矩
 * @param comp_right 输出：右腿补偿力矩
 */
  void LegPredictor_GetCompensation( float *comp_left,float *comp_right)
{
	if (comp_left && comp_right) {
		*comp_left = leg_predictor.comp_left;
		*comp_right = leg_predictor.comp_right;
	}
}