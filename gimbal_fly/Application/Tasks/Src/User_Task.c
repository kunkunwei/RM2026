#include "main.h"
#include "user_task.h"

#include "Gimbal_task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"
#include "Can_Task.h"
#include "usart.h"
#include "vofa.h"

void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);

    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;
    extern SBUS_Remote_Info_Typedef sbus_remote_ctrl;

    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t* Usb_receive_data;

    extern dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor; 
    extern dm_motor_measure_t pitch_motor;
    extern dm_motor_measure_t yaw_motor;

    const  gimbal_t* locaal_gimbal=get_gimbal_point();
    extern uint8_t Usart_Mode;
    extern refree_info_t refree_info;
    for(;;)
    {   
        // Vofa_Send_RC(&huart1, &sbus_remote_ctrl);
        // Vofa_Send_INS(&huart1,INS_Info,ist8310_Info);
        // Vofa_Send_Gimbal(&huart1,gimbal);
        //usbDebug_float(6.89);
        Vofa_Send_shoot_Info(&huart6,locaal_gimbal);
#ifdef USE_SBUS_PROTOCOL
        if ((sbus_remote_ctrl.rc.s[0]==-1)&&(remote_ctrl.rc.s[1]==-1))
        {
            buzzer_off();
            // buzzer=1;
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((sbus_remote_ctrl.rc.s[0]==1)&&(remote_ctrl.rc.s[1]==-1))
        {
            // buzzer_on(84,10);
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((remote_ctrl.rc.s[1]==1))
        {

            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#else
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

            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#endif
        // // 检测
        //  if( gimbal.gimbal_pos.yaw_motor_measure->err != 1){
        //    uart_printf(&huart6,"YAW:离线 ERR:%d\r\n",gimbal.gimbal_pos.yaw_motor_measure->err);
        //    Damiao_Motor_Enable(0x01);
        //  }
        // if( gimbal.gimbal_pos.pitch_motor_measure->err != 1){
        //     uart_printf(&huart6,"PITCH:离线 ERR:%d\r\n",gimbal.gimbal_pos.pitch_motor_measure->err);
        //     Damiao_Motor_Enable(0x02);
        // }

        osDelay(50);
    }
}
