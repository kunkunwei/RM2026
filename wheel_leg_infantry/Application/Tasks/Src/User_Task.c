#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bmi088.h"
#include "bsp_tim.h"
#include "buzzer_music.h"
#include "chassis_behaviour.h"
#include "ctl_chassis.h"
#include "monitor.h"
#include "usart.h"
#include "vofa.h"



// void Test_MagYaw(ist8310_real_data_t *ist8310_Info,INS_Info_Typedef *INS_Info);
void User_Task(void const * argument)
{
  /* Infinite loop */

    osDelay(800);
    TickType_t systick = 0;

    static float last_refresh_dog_time = 0.0f;
    static float last_led_time = 0.0f;
    float current_time = 0.0f;
    float last_RC_time = 0.0f;
    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;
    extern Chassis_RC_Info_t chassis_can_rc_info;
    // extern gimbal_chassis_comm_t g_comm ;
    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t* Usb_receive_data;

    extern lk9025_motor_measure_t motor_right, motor_left;
    extern dm8009_motor_measure_t motor_joint[4];
    const SlipDetector_t *local_detector= get_slip_detector_point();
    const chassis_move_t* local_chassis = get_chassis_control_point();
    const Quaternion_Info_Typedef* local_Quaternion_Info = get_quaternion_info_point();
    const LegPredictor_t *leg_predictor = get_leg_predictor_point();
    const PC_Ctrl_Info_t *pc_ctrl_info = get_pc_uart_ctrl_point();
    //监测
    const SystemMonitor_t *monitor = SystemMonitor_Get();


    chassis_comm_init();
    const gimbal_ctrl_frame_t *gimbal_ctl_point = chassis_get_ctl();
    for(;;)
    {
        systick = osKernelSysTick();
        current_time=DWT_GetTimeline_ms();
        // uart_printf(&huart1,"ctl: %d %d %d %d \r\n",
        //     gimbal_ctl_point->ctrl_flags,
        //     gimbal_ctl_point->gimbal_yaw,
        //     gimbal_ctl_point->target_speed_x,
        //     gimbal_ctl_point->target_speed_wz);

        // Vofa_Send_System(&huart1,local_chassis);
        // Vofa_Send_PC_Ctrl_Info(&huart1,pc_ctrl_info);
        // Vofa_Send_joint_angle(&huart1,local_chassis);
        // Vofa_Send_1motor_Data(&huart1,local_chassis);
        Vofa_Send_New_Chassis_Data(&huart1,local_chassis);
        // if (current_time-last_RC_time>10.0f)
        // {
        //     update_gimbal_comm_status(current_time);
        //     get_safe_gimbal_cmd(&gimbal_chassis_comm.gimbal_cmd,current_time);
        //     chassis_data_to_gimbal_feedback(local_chassis);
        //     uart_printf(&huart1, "frame1_received:%d,frame2_received:%d,comm_ok:%d,safe_mode:%d\r\n",gimbal_chassis_comm.frame1_received,gimbal_chassis_comm.frame2_received,gimbal_chassis_comm.comm_ok,gimbal_chassis_comm.safe_mode);
        // }

        //每隔1s就要喂狗，防止1.5s看门狗复位
        // if ((current_time-last_refresh_dog_time)>=800)
        // {
        //     last_refresh_dog_time=current_time;
        //     SystemMonitor_WatchdogRefresh();
        //
        // }
        // LED更新（每50ms调用一次，即每25个循环）
        // led_update_counter++;
        if ((current_time-last_led_time)>=50)
        {
            last_led_time=current_time;
            SystemMonitor_PeriodicTask();


        }
        // uart_printf(&huart1,"ecd:%d,pos:%.3f,speed:%.2f,current:%d,set:%.2f\r\n",local_chassis->right_leg.front_joint.joint_motor_measure->ecd,
        // local_chassis->right_leg.front_joint.joint_motor_measure->pos,
            // local_chassis->right_leg.front_joint.joint_motor_measure->speed,
            // local_chassis->right_leg.front_joint.joint_motor_measure->current,
              // local_chassis->right_leg.front_joint.current_set);
        // Vofa_Send_joint_angle(&huart1,local_chassis);
        // Vofa_Send_Tor(&huart1,local_chassis);
        // chassis_send_feedback(local_chassis);

        // Vofa_Send_Chassis_CMD(&huart1, &gimbal_chassis_comm,local_chassis);

        // cpu_d=Monitor_GetCPULoad();
            // uart_printf(&huart6, "heart_cont:%d\r\n", monitor_heartbeat->heartbeat_count);
            // uart_printf(&huart6, "heart_status:%d\r\n", monitor_heartbeat->status);
            // uart_printf(&huart6, "heart_restart_reason:%d\r\n", monitor_heartbeat->restart_reason);
            // uart_printf(&huart6, "watchdog_restart_count:%d\r\n", monitor_heartbeat->watchdog_restart_count);
            // uart_printf(&huart6, "cpu:%f\r\n", monitor->cpu.cpu_load_percent);

        // Monitor_Watchdog_Refresh();
        // Test_MagYaw(&ist8310_Info,&INS_Info);
        //printf("%.2f,%.2f\r\n",local_chassis->chassis_roll,local_chassis->chassis_roll_set);
        // Vofa_Send_INS(&huart6,INS_Info,ist8310_Info);
        // Vofa_Send_Q(&huart6,INS_Info,local_Quaternion_Info);
        // Vofa_Send_Chassis(&huart6,INS_Info,motor_joint,local_chassis);


        // Vofa_Send_Chassis_CMD(&huart1, &chassis_can_rc_info,local_chassis);
        // Vofa_Send_Data(&huart1,local_chassis);

        // Vofa_Send_Calibrate(&huart6,local_chassis);
        // Vofa_Send_Slip(&huart6,local_chassis,local_detector);
        // Vofa_Send_Balance(&huart6,local_chassis);
        // Vofa_Send_Pred(&huart1,local_chassis);
        // Vofa_Send_Theata(&huart6,local_chassis);
        // Vofa_Send_Theata_pre(&huart6,local_chassis ,leg_predictor,local_detector);

        if (switch_is_down(remote_ctrl.rc.s[0]))
        {
            buzzer_off();
            // buzzer=1;
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

        osDelayUntil(&systick, 50); // 50ms周期控制
    }
}

