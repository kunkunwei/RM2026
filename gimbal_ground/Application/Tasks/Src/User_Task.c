#include "main.h"
#include "user_task.h"

#include "Gimbal_task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"
#include "Can_Task.h"
#include "usart.h"
#include "vofa.h"

void User_Task(void const *argument)
{
    /* 等待云台初始化完成，避免访问空指针 */
    while (!is_gimbal_init_done()) osDelay(1);

    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t *Usb_receive_data;
    extern refree_info_t refree_info;
    const PC_Ctrl_Info_t *local_pc_ctrl_info= get_pc_uart_ctrl_point();
    for (;;)
    {
        // Vofa_Send_PC_Ctrl_Info(&huart6,local_pc_ctrl_info);
        // Vofa_Send_INS(&huart1,*get_ins_info_point(),ist8310_Info);

        // Vofa_Send_Motorr(&huart6, get_gimbal_point());
        // Vofa_Send_shoot_Info(&huart6, get_gimbal_point());
#ifdef   USE_PC_CONTROL
        if (local_pc_ctrl_info->rc.mode_sw==1)
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (local_pc_ctrl_info->rc.mode_sw==2)
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#endif
#ifdef USE_SBUS_PROTOCOL
        if ((get_sbus_remote_control_point()->rc.s[LEFT_1_SWITCH] == -1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((get_sbus_remote_control_point()->rc.s[LEFT_1_SWITCH] == 1) && (get_remote_control_point()->rc.s[LEFT_2_SWITCH] == -1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if ((get_remote_control_point()->rc.s[LEFT_1_SWITCH] == 1))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        }
#else
        if (switch_is_down(get_remote_control_point()->rc.s[0]))
        {
            buzzer_off();
            // buzzer = 1;
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_up(get_remote_control_point()->rc.s[0]))
        {
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        }
        else if (switch_is_mid(get_remote_control_point()->rc.s[0]))
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
