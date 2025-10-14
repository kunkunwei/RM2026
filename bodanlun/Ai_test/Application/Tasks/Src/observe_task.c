#include "observe_task.h"
#include "chassis_task.h"
KalmanFilter_Info_TypeDef vaEstimateKF;	  

float vaEstimateKF_F[4] = {1.0f, 0.005f, 
                           0.0f, 1.0f};	   

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    

float vaEstimateKF_Q[4] = {5.0f, 0.0f, 
                           0.0f, 5.0f};    

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  300.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	
														 															 

float vel_acc[2]; 
uint32_t OBSERVE_TIME=5;
fp32 v_real;
fp32 aver_v;
fp32 diff_v;
fp32 body_v;
// fp32 x_real;

//extern const chassis_move_t* local_chassis_move;
const chassis_move_t* local_chassis_move;
extern const fp32 *local_imu_gyro;	  //imu数据		
//extern bool_t Robot_Offground_detect(chassis_move_t *chassis_move_detect);
void ObserveTask(void const * argument)
{
  // const volatile fp32 *angle;
  // const volatile fp32 *gyro;

  // angle = get_INS_angle_point();
  // gyro = get_gyro_data_point();


	// while(local_chassis_move->touchingGroung == false || local_chassis_move->is_conversely == true)
	// {
	//   //st
	// }

	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	//static float aver_v=0.0f;
  static float last_v=0.0f;
	xvEstimateKF_Init(&vaEstimateKF);
	
  TickType_t systick = 0;
  while(1)
	{  
		osDelayUntil(&systick,5);

    
	}
}

void xvEstimateKF_Init(KalmanFilter_Info_TypeDef *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 观测维度
	
		memcpy(EstimateKF->Data.A, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->Data.P, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Data.Q, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->Data.R, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->Data.H, vaEstimateKF_H, sizeof(vaEstimateKF_H));

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
