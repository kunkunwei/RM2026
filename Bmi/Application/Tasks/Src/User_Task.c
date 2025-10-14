#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "arm_math.h"
#include "bmi088.h"
#include "bsp_tim.h"
#include "INS_Task.h"
#include "ist8310.h"
#include "usart.h"
#include "vofa.h"

void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);
    TickType_t systick = 0;
    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;

    // extern Usb_data_fliter_t usb_fliter_data;
    // extern Usb_dpkg_data_t* Usb_receive_data;

    extern lk9025_motor_measure_t motor_right, motor_left;
    // extern dm8009_motor_measure_t motor_joint[4];
    
    // const chassis_move_t* local_chassis = get_chassis_control_point();
    fp32 mag[3]={0,0,0};
    extern BMI088_Info_Typedef BMI088_Info;
    extern  chassis_move_t chassis_move;
		//float leg_cal[2];
    //Usb_send_data_t Usb_send_data_t;
    //Usb_dpkg_data_t* cnm = getUsbDpkgData();

    // float wz,wz_fliter;
    // const float num = 0.2f;
    // first_order_filter_type_t filter_t = {.input = wz, .frame_period=0.05f, .out=wz_fliter, .num=num};
    // first_order_filter_init(&filter_t,0.05f,&num);
    for(;;)
    {
        systick = osKernelSysTick();
        // Vofa_SendRCChannels(&huart1,&BMI088_Info);
        // Vofa_SendRCChannels(&huart1,&INS_Info);
        ist8310_read_mag(mag);
        // Vofa_Send(&huart1,&BMI088_Info,mag);
        // Vofa_SendChassis(&huart1,&chassis_move);
        Vofa_SendIMU(&huart1,&INS_Info,mag);
        if (switch_is_down(remote_ctrl.rc.s[0]))
        {

            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_up(remote_ctrl.rc.s[0]))
        {
            // buzzer_on(84,10);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_mid(remote_ctrl.rc.s[0]))
        {

            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
        // printf("%.2f,%.2f\r\n",local_chassis->right_leg.leg_angle,local_chassis->left_leg.leg_angle);
        osDelay(50);
    }
}
