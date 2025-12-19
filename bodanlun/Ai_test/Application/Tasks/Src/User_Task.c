#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"
#include "buzzer_music.h"
#include "usart.h"
#include "usb.h"
#include "vofa.h"
void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);
    uint16_t buzzer=1;
    TickType_t systick = 0;
    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;

    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t* Usb_receive_data;
    extern control_mode_t control_mode;
    extern lk9025_motor_measure_t motor_right, motor_left;
    // extern dm8009_motor_measure_t motor_joint[4];

    // const chassis_move_t* local_chassis = get_chassis_control_point();
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
        // uart_printf(&huart1,"crc_suss ");
        // buzzer_on(84,10);
        //printf("%.2f,%.2f\r\n",local_chassis->chassis_roll,local_chassis->chassis_roll_set);
        // Vofa_Send_Chassis(&huart1,INS_Info);
        // if (systick<10000)buzzer_on(84,3000);
        // else buzzer_off();
        // if (systick<10000) play_music(jntm_score, sizeof(jntm_score) / sizeof(jntm_score[0]));
        // else buzzer_off();
        if (switch_is_down(remote_ctrl.rc.s[0]))
        {
            buzzer_off();
            buzzer=1;
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
            // buzzer_on(84,100);
            // if (buzzer>0)
            // {
            //     buzzer=0;
            //     buzzer_on(84,10);
            // }
            // else
            // {
            //     buzzer_off();
            // }
            // if (systick<10000)buzzer_on(84,10);
            // else buzzer_off();
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
        // uart_printf(&huart1,"crc_suss ;\r\n");
        // if (switch_is_mid(remote_ctrl.rc.s[1]))
        // {
        //     // buzzer_on(84,10);
        //     play_music(jntm_score, sizeof(jntm_score) / sizeof(jntm_score[0]));
        //
        // }
        // printf("%.2f,%.2f\r\n",local_chassis->right_leg.leg_angle,local_chassis->left_leg.leg_angle);
        osDelay(10);
    }
}
