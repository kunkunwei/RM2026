#ifndef __ROS_TASK__H__
#define __ROS_TASK__H__
#include "main.h"


#define ROS_START_BYTE 0x42
#define ROS_RX_BUF_NUM 32u
#define ROS_FRAME_LENGTH 16u

//陀螺仪数据发送周期 ms
#define IMU_SEND_TIME 20

typedef struct
{
    fp32 shoot_yaw;
    fp32 shoot_pitch;
    fp32 shoot_depth;
}ROS_Msg_t;
void Ros_Task(void const * argument);

#endif