#include "main.h"
#include "user_task.h"
#include "Chassis_Task.h"
#include "Gimbal_task.h"
#include "observe_task.h"
#include "arm_math.h"
void User_Task(void const * argument)
{
  /* Infinite loop */
    osDelay(800);

    extern INS_Info_Typedef INS_Info;
    extern Remote_Info_Typedef remote_ctrl;



    for(;;)
    {   

        

        //printf("%.2f,%.2f,%.2f\r\n",feibiao.tmp_yaw,feibiao.next_yaw_pos,feibiao.yaw_6020->real_pos);
        osDelay(50);
    }
}