#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "Gimbal_task.h"
#include "observe_task.h"
#include "arm_math.h"
#include "Can_Task.h"
void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);

    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;

    extern Usb_data_fliter_t usb_fliter_data;
    extern Usb_dpkg_data_t* Usb_receive_data;

    extern dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor; 
    extern dji_motor_measure_t pitch_motor;
    extern dm_motor_measure_t yaw_motor;

    extern gimbal_t gimbal;
    extern uint8_t Usart_Mode;
    extern refree_info_t refree_info;
    for(;;)
    {   

        //usbDebug_float(6.89); 
        //printf("%d,%d,%d,%d\r\n",refree_info.Enerage_buffer,refree_info.GameState,refree_info.Hp_Percentage,refree_info.Shoot_Heat_Percentage);
        // printf("%.2f,%.2f\r\n",gimbal.ins_info->pit_angle,\
        //                           gimbal.ins_info->pit_gyro);

        // printf("%.2f,%.2f,%d\r\n",gimbal.gimbal_pos.pitch_absolute_pos,\
        //                         gimbal.gimbal_pos.pitch_target_pos,\
        //                           gimbal.gimbal_pos.pitch_motor_measure->target_current);

        // printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_pos.gimbal_motor_gyro_pid.Iout,\
        //   gimbal.gimbal_pos.gimbal_motor_gyro_pid.Pout,\
        //   gimbal.gimbal_pos.gimbal_motor_gyro_pid.Dout);
        // printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_pos.yaw_relattive_pos,yaw_motor.pos,\
        //           yaw_motor.deta_yaw_pos);

        //printf("%.2f,%.2f,%.2f\r\n",yaw_motor.pos,yaw_motor.last_yaw_pos,yaw_motor.deta_yaw_pos);
        
        // printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_pos.yaw_absolute_pos,gimbal.gimbal_pos.yaw_target_pos,\
        //                             gimbal.gimbal_pos.yaw_motor_measure->target_tor);
        //printf("%d\r\n",gimbal.gimbal_mod);

        printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_shoot.shoot_left_rad,\
                                  gimbal.gimbal_shoot.shoot_right_rad,\
                                  gimbal.gimbal_shoot.shoot_pull_rad);

        // printf("%d,%d,%d\r\n",gimbal.gimbal_shoot.shoot_motor_left->target_current,\
        //       gimbal.gimbal_shoot.shoot_motor_right->target_current,\
        //       gimbal.gimbal_shoot.pull_motor->target_current);
        //printf("%.2f,%d\r\n", gimbal.gimbal_shoot.shoot_pull_rad,gimbal.gimbal_shoot.pull_is_block);
        //printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_shoot.Shoot_right_PID.error[0],gimbal.gimbal_shoot.Shoot_right_PID.Kp,gimbal.gimbal_shoot.Shoot_right_PID.Pout);
        
        // printf("%d,%.2f,%.2f\r\n",gimbal.gimbal_mod,gimbal.gimbal_pos.yaw_motor_measure->target_tor,\
        //                 gimbal.gimbal_pos.yaw_motor_measure->speed);
        // printf("%.2f,%d\r\n",gimbal.gimbal_shoot.shoot_right_rad,gimbal.gimbal_shoot.shoot_motor_right->target_current);
        //printf("%.2f,%.2f\r\n",0.6f,gimbal.ins_info->yaw_gyro);
        

        //printf("%.2f,%.2f\r\n",Usb_receive_data->vx_set,Usb_receive_data->vy_set);
        // printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_pos.yaw_absolute_pos,\
        //                             gimbal.gimbal_pos.usb_autoAim_ptr->minipc_target_yaw,\
        //                             gimbal.gimbal_pos.yaw_target_pos);

        // printf("%.2f,%.2f,%.2f\r\n",gimbal.gimbal_pos.pitch_absolute_pos,\
        //                             gimbal.gimbal_pos.usb_autoAim_ptr->minipc_target_pitch,\
        //                             gimbal.gimbal_pos.pitch_target_pos);
        //printf("%.2f,%.2f,%d\r\n",gimbal.gimbal_pos.tmp,gimbal.ins_info->pit_gyro,gimbal.gimbal_pos.pitch_motor_measure->target_current);
        //printf("%d\r\n",gimbal.gimbal_mod);
        //printf("Mode : %d,%.2f\r\n",gimbal.gimbal_mod,gimbal.gimbal_pos.yaw_target_pos);
        //printf("MOD;%d\r\n",gimbal.gimbal_pos.usb_autoAim_ptr->autoMod);
        // printf("UART:MOD:%d,GIMBAL_MOD:%d,ERR:%d,Shoot:%d\r\n",Usart_Mode,gimbal.gimbal_mod,gimbal.gimbal_pos.yaw_motor_measure->err,\
        //   gimbal.gimbal_pos.usb_autoAim_ptr->shoot_flag);
        //检测
        if( gimbal.gimbal_pos.yaw_motor_measure->err != 1){
          printf("YAW:离线 ERR:%d\r\n",gimbal.gimbal_pos.yaw_motor_measure->err);
          Damiao_Motor_Enable();
        }

        osDelay(50);
    }
}