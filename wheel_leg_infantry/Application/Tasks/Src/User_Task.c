#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "bmi088.h"
#include "bsp_tim.h"
#include "buzzer_music.h"
#include "ctl_chassis.h"
#include "monitor.h"
#include "protocol.h"
#include "referee.h"
#include "usart.h"
#include "vofa.h"


void User_Task(void const * argument)
{
  /* Infinite loop */

    osDelay(800);
    TickType_t systick = 0;


    static float last_led_time = 0.0f;
    float current_time = 0.0f;
    //
    // extern INS_Info_Typedef INS_Info;
    // extern Remote_Info_Typedef remote_ctrl;
    // extern Chassis_RC_Info_t chassis_can_rc_info;
    // // extern gimbal_chassis_comm_t g_comm ;
    // extern Usb_data_fliter_t usb_fliter_data;
    // extern Usb_dpkg_data_t* Usb_receive_data;
    //
    // extern lk9025_motor_measure_t motor_right, motor_left;
    // extern dm8009_motor_measure_t motor_joint[4];
    // const SlipDetector_t *local_detector= get_slip_detector_point();
    const chassis_move_t* local_chassis = get_chassis_control_point();
    // const Quaternion_Info_Typedef* local_Quaternion_Info = get_quaternion_info_point();
    // const LegPredictor_t *leg_predictor = get_leg_predictor_point();
    // const PC_Ctrl_Info_t *pc_ctrl_info = get_pc_uart_ctrl_point();
    // const gimbal_ctrl_frame_t *gimbal_ctl_point = chassis_get_ctl();
    //监测
    const SystemMonitor_t *monitor = SystemMonitor_Get();
    chassis_comm_init();

    for(;;)
    {
        systick = osKernelSysTick();
        current_time=DWT_GetTimeline_ms();

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


        chassis_send_feedback(local_chassis);

        osDelayUntil(&systick, 50); // 50ms周期控制
    }
}

