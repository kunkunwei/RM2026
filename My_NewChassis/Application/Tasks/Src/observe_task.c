/**
  ******************************************************************************
  * @file           : observe_task.c
  * @brief          : 观测任务实现文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 实现轮足机器人状态观测，包括速度估计、打滑检测等功能
  *                  采用卡尔曼滤波融合IMU加速度和轮速计数据
  ******************************************************************************
  */

#include "observe_task.h"
#include "chassis_task.h"

/* 卡尔曼滤波器实例 */
KalmanFilter_Info_TypeDef vaEstimateKF;	  

/**
 * @brief 状态转移矩阵F (2x2)
 * @note F = [1, dt; 0, 1] 其中dt=0.005s(200Hz)
 *       状态向量: [position, velocity]
 */
float vaEstimateKF_F[4] = {1.0f, 0.005f, 
                           0.0f, 1.0f};	   

/**
 * @brief 协方差矩阵P初值 (2x2)
 */
float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    

/**
 * @brief 过程噪声协方差矩阵Q (2x2)
 * @note 调节对系统模型的信任度
 */
float vaEstimateKF_Q[4] = {5.0f, 0.0f, 
                           0.0f, 5.0f};    

/**
 * @brief 观测噪声协方差矩阵R (2x2)
 * @note 调节对传感器测量的信任度，会根据打滑情况动态调整
 */
float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  300.0f}; 	
														
float vaEstimateKF_K[4];  // 卡尔曼增益矩阵
													 
/**
 * @brief 观测矩阵H (2x2)
 * @note H = [1, 0; 0, 1] 直接观测位置和速度
 */
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	
														 															 

float vel_acc[2];             // 卡尔曼滤波输出: [位置, 速度]
uint32_t OBSERVE_TIME=5;      // 观测任务周期(ms)
fp32 v_real;                  // 实际速度(考虑腿部运动补偿)
fp32 aver_v;                  // 平均速度(左右轮速度平均)
fp32 diff_v;                  // 差速(用于检测打滑)
fp32 body_v;                  // 机体速度(卡尔曼滤波输出)
// fp32 x_real;  // 位置积分值(已注释)

//extern const chassis_move_t* local_chassis_move;
//const chassis_move_t* local_chassis_move;
extern const fp32 *local_imu_gyro;	  // IMU陀螺仪数据指针		
//extern bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);

/**
 * @brief  观测任务主函数
 * @param  argument FreeRTOS任务参数(未使用)
 * @retval None
 * @note   以5ms周期运行，进行速度观测和状态估计
 */
void ObserveTask(void const * argument)
{
  // const volatile fp32 *angle;  // 姿态角度指针(已注释)
  // const volatile fp32 *gyro;   // 陀螺仪数据指针(已注释)

  // angle = get_INS_angle_point();  // 获取INS角度数据(已注释)
  // gyro = get_gyro_data_point();   // 获取陀螺仪数据(已注释)

  local_chassis_move = get_chassis_control_point();  // 获取底盘控制数据指针
	// 等待机器人着地且不处于倒置状态(已注释)
	// while(local_chassis_move->touchingGroung == false || local_chassis_move->is_conversely == true)
	// {
	//   //st
	// }

	static float wr,wl=0.0f;     // 右轮、左轮角速度
	static float vrb,vlb=0.0f;   // 右轮、左轮线速度(机体坐标系)
	//static float aver_v=0.0f;   // 平均速度(已注释)
  static float last_v=0.0f;    // 上次速度值
	xvEstimateKF_Init(&vaEstimateKF);  // 初始化卡尔曼滤波器
	
  TickType_t systick = 0;  // 系统时钟计数
  while(1)
	{  
		osDelayUntil(&systick,5);  // 5ms周期延时
		// printf("test\r\n");  // 调试输出(已注释)
		
		/* 计算右轮角速度(包含腿部关节运动补偿) */
		// 完整版本(包含IMU陀螺仪补偿，已注释):
		// wr =    -local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
    //         + *(gyro+INS_GYRO_Y_ADDRESS_OFFSET) \
    //         + local_chassis_move->right_leg.angle_dot;
    wr =    -local_chassis_move->right_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
            + local_chassis_move->right_leg.angle_dot;  // 轮速 + 腿部角速度补偿
            
    /* 计算右轮在机体坐标系下的线速度 */        
    vrb=    wr*WHEEL_R \  // 轮子转动产生的速度分量
            +local_chassis_move->right_leg.leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta)\ // 腿部摆动速度分量
            +local_chassis_move->right_leg.length_dot*arm_sin_f32(local_chassis_move->state_ref.theta);  // 腿长变化速度分量
    
    /* 计算左轮角速度(包含腿部关节运动补偿) */
    // 完整版本(包含IMU陀螺仪补偿，已注释):
    // wl =     local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
    //          + *(gyro+INS_GYRO_Y_ADDRESS_OFFSET) \
    //          + local_chassis_move->left_leg.angle_dot;

    wl =     local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed/57.3f \
             + local_chassis_move->left_leg.angle_dot;  // 轮速 + 腿部角速度补偿

    /* 计算左轮在机体坐标系下的线速度 */
    vlb=    wl*WHEEL_R \  // 轮子转动产生的速度分量
                +local_chassis_move->left_leg.leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta)\ // 腿部摆动速度分量
                +local_chassis_move->left_leg.length_dot*arm_sin_f32(local_chassis_move->state_ref.theta);  // 腿长变化速度分量
                
   //printf("%f,%f\r\n",wl,vlb);  // 调试输出(已注释)
   
    /* 计算平均速度和速度变化 */
    last_v = aver_v;              // 保存上次速度
    aver_v=(vrb+vlb)/2.0f;        // 计算左右轮平均速度

    //printf("%f,%f,%f,\r\n",local_chassis_move->left_leg.wheel_motor.wheel_motor_measure->speed_rpm/2160.0f,-*(gyro+INS_GYRO_Y_ADDRESS_OFFSET),-local_chassis_move->left_leg.angle_dot);
    
    /* 计算差速，用于检测打滑 */
    diff_v = (wr - wl)*WHEEL_R - local_chassis_move->wz_set*2*MOTOR_DISTANCE_TO_CENTER;
    
    /* 根据差速情况动态调整观测噪声协方差 */
    if(diff_v >2.0f || diff_v <-2.0f){
      // 检测到打滑，降低对速度测量的信任度
      vaEstimateKF.Data.R[0] = 5000.0f;  // 增大观测噪声方差
    }
    else{
      // 正常情况，保持标准信任度
      vaEstimateKF.Data.R[0] = 250.0f;   // 标准观测噪声方差
    }
    
    /* 更新卡尔曼滤波器 */
    xvEstimateKF_Update(&vaEstimateKF,*(local_chassis_move->chassis_imu_accel+INS_ACCEL_X_ADDRESS_OFFSET) - 0.4f,aver_v);
    // 备选方案(使用数值微分加速度，已注释):
    //xvEstimateKF_Update(&vaEstimateKF,(aver_v - last_v)/0.005,aver_v);
    //printf("%f,%f\r\n",(*(local_chassis_move->chassis_imu_accel + INS_ACCEL_X_ADDRESS_OFFSET)) - 0.4,aver_v);

    /* 计算不同类型的速度输出 */
    //v_real = vel_acc[0];  // 直接使用滤波输出(已注释)
    
    body_v = vel_acc[0];  // 机体速度：卡尔曼滤波输出的速度
    
    /* 实际速度：机体速度减去腿部摆动产生的速度分量 */
    v_real = vel_acc[0] \
                -local_chassis_move->leg_length*local_chassis_move->state_ref.theta_dot*arm_cos_f32(local_chassis_move->state_ref.theta);
                // 备选补偿项(已注释): -local_chassis_move->leg_length*arm_sin_f32(local_chassis_move->state_ref.theta);

    /* 位置积分和角速度计算(已注释) */
    //10 ms
    // x_real;  // 位置积分
    // x_real += v_real * 0.005f;  // 位置积分更新
    //fp32 wz = 0.5f * (wr - wl)*WHEEL_R / MOTOR_DISTANCE_TO_CENTER;  // 角速度计算

    //printf("%f,%f\r\n",v_real,x_real);  // 调试输出(已注释)
    
    /* 动态调整观测噪声协方差矩阵的备用代码(已注释) */
    // rebuild matrix R
    //kf->R_data[i * kf->MeasurementValidNum + i] = kf->MatR_DiagonalElements[kf->temp[i]];
    
	}
}

/**
 * @brief  卡尔曼滤波器初始化函数
 * @param  EstimateKF 卡尔曼滤波器结构体指针
 * @retval None
 * @note   初始化2维状态(位置+速度)，2维观测(速度+加速度)的卡尔曼滤波器
 */
void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF)
{
    /* 初始化卡尔曼滤波器参数 */
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态维度=2, 控制维度=0, 观测维度=2
	
		/* 复制预设的矩阵参数 */
		memcpy(EstimateKF->Data.A, vaEstimateKF_F, sizeof(vaEstimateKF_F));  // 状态转移矩阵
    memcpy(EstimateKF->Data.P, vaEstimateKF_P, sizeof(vaEstimateKF_P));  // 协方差矩阵初值
    memcpy(EstimateKF->Data.Q, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));  // 过程噪声协方差
    memcpy(EstimateKF->Data.R, vaEstimateKF_R, sizeof(vaEstimateKF_R));  // 观测噪声协方差
    memcpy(EstimateKF->Data.H, vaEstimateKF_H, sizeof(vaEstimateKF_H));  // 观测矩阵

}

/**
 * @brief  卡尔曼滤波器更新函数
 * @param  EstimateKF 卡尔曼滤波器结构体指针
 * @param  acc        加速度测量值(m/s²)
 * @param  vel        速度测量值(m/s)
 * @retval None
 * @note   使用速度和加速度观测值更新滤波器状态估计
 */
void xvEstimateKF_Update(KalmanFilter_Info_TypeDef *EstimateKF ,float acc,float vel)
{   	
    /* 设置观测向量 */
    EstimateKF->MeasuredVector[0] =	vel;  // 速度观测
    EstimateKF->MeasuredVector[1] = acc;  // 加速度观测
    		
    /* 执行卡尔曼滤波更新 */
    Kalman_Filter_Update(EstimateKF);

    /* 提取滤波输出结果 */
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->Output[i];  // vel_acc[0]=位置, vel_acc[1]=速度
    }
}
/**
 * @brief  获取卡尔曼滤波后的实际速度
 * @retval fp32 实际速度(m/s)，已去除腿部摆动影响
 */
fp32 get_KF_Spd(void)
{   	
    return v_real;
}

/**
 * @brief  获取原始平均速度
 * @retval fp32 左右轮速度平均值(m/s)
 */
fp32 get_raw_Spd(void)
{   	
    return aver_v;
}

/**
 * @brief  获取差速值
 * @retval fp32 差速(m/s)，用于打滑检测
 */
fp32 get_diff_Spd(void)
{   	
    return diff_v;
}

/**
 * @brief  获取机体速度
 * @retval fp32 机体速度(m/s)，卡尔曼滤波输出
 */
fp32 get_body_Spd(void)
{   	
    return body_v;
}
