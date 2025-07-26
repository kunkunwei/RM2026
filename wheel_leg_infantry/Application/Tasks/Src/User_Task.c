#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bsp_tim.h"

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

    extern lk9025_motor_measure_t motor_right, motor_left;
    extern dm8009_motor_measure_t motor_joint[4];
    
    const chassis_move_t* local_chassis = get_chassis_control_point();
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
        //printf("%.2f,%.2f\r\n",usb_fliter_data.vx_after_fliter,usb_fliter_data.wz_after_fliter);
        //原始ins数据
        //printf("%.2f,%.2f%.2f\r\n",local_chassis->chassis_imu_gyro[INS_GYRO_X_ADDRESS_OFFSET],local_chassis->chassis_imu_gyro[INS_GYRO_Y_ADDRESS_OFFSET],local_chassis->chassis_imu_gyro[INS_GYRO_Z_ADDRESS_OFFSET]);
        //腿长debug
        //printf("%.2f,%.2f\r\n",local_chassis->right_leg.leg_length,local_chassis->left_leg.leg_length);
        //姿态debug
        //printf("pitch:%.2f,yaw:%.2f,roll%.2f\r\n",local_chassis->chassis_pitch,local_chassis->chassis_yaw,local_chassis->chassis_roll);
        //printf("%.2f,%.2f\r\n",local_chassis->state_ref.phi,local_chassis->state_ref.phi_dot);
        //printf("%.2f,%.2f\r\n",local_chassis->state_ref.phi,local_chassis->state_ref.phi_dot);
        //速度debug
        //printf("%.2f,%.2f\r\n",local_chassis->state_ref.x_dot,get_body_Spd());
        //输出debug
        // printf("%.2f,%.2f,%.2f,%.2f\r\n",local_chassis->right_leg.front_joint.tor_set,\
        //                                  local_chassis->right_leg.back_joint.tor_set,\
        //                                  local_chassis->left_leg.front_joint.tor_set,\
        //                                  local_chassis->left_leg.back_joint.tor_set);
        //printf("%d,%d\r\n",local_chassis->right_leg.wheel_motor.give_current,local_chassis->left_leg.wheel_motor.give_current);                              
        //
        //
        //printf("%.2f\r\n",local_chassis->tmp);
        //printf("%.2f,%.2f,%.2f,%.2f\r\n",STOP_X_OFFSET,local_chassis->state_ref.x,local_chassis->state_set.x_dot,local_chassis->state_ref.x_dot);
        //printf("%.2f,%.2f\r\n",local_chassis->wz_from_ros,local_chassis->vx_from_ros);
        //printf("%.2f\r\n",local_chassis->state_set.x_dot);
        //printf("%.2f\r\n",Usb_receive_data->wz_set);
        //printf("%.2f,%.2f\r\n",Usb_receive_data->vx_set,Usb_receive_data->wz_set);
        //printf("%.2f,%.2f\r\n",local_chassis->wz_from_ros,*(local_chassis->chassis_imu_gyro+INS_GYRO_Z_ADDRESS_OFFSET));
        //printf("%.2f,%.2f\r\n",local_chassis->chassis_roll,local_chassis->chassis_roll_set);

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
            // buzzer_on(100);
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
        printf("%.2f,%.2f\r\n",local_chassis->right_leg.leg_angle,local_chassis->left_leg.leg_angle);
        osDelay(50);
    }
}
