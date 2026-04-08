#include "Ros_Task.h"

#include "ctl_chassis.h"
#include "observe_task.h"
#include "usb.h"
#include "minipc.h"
#include "Gimbal_task.h"
#include "usart.h"
#include "vofa.h"
Usb_send_data_t Usb_send_data;
const Usb_dpkg_data_t *Usb_receive_data = NULL;
Usb_data_fliter_t usb_fliter_data;

/* gimbal 已迁移至 static，通过 get_gimbal_point() 只读访问 */

/////////
// static void Low_Cbroad_Can_Send();
// static void high_Cbroad_Can_Send();
// static void yaokong_Cbroad_Can_Send();

extern CAN_TxFrameTypeDef LOW_CBROAD_FRAME;
extern refree_info_t refree_info;
/////
static void ros_send_gimbal_attitude(void);

void Ros_Task(void const *argument)
{
  /* 等待云台初始化完成，避免访问空指针 */
  while (!is_gimbal_init_done())
    osDelay(1);
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */

  for (;;)
  {
    ros_send_gimbal_attitude();

    // 下方注释中的调试输出可改用 get_gimbal_point() 和 gimbal_get_feedback()
    // uart_printf(&huart1,"comm_ok:%d\r\n,...", gimbal_get_feedback()->...);

    vTaskDelay(50);
  }
  /* USER CODE END Ros_Task */
}

static void ros_send_gimbal_attitude(void)
{
  const gimbal_t *gimbal = get_gimbal_point();

  if (gimbal == NULL || gimbal->ins_info == NULL)
  {
    return;
  }

  MiniPC_SendGimbalAttitude(gimbal->ins_info->yaw_angle,
                            gimbal->ins_info->pit_angle,
                            gimbal->ins_info->rol_angle);
}

static void Low_Cbroad_Can_Send()
{

  /* 使用memcpy直接拷贝浮点数的内存布局 */
  // LOW_CBROAD_FRAME.Data[0]=1;
  // LOW_CBROAD_FRAME.Data[1]=2;LOW_CBROAD_FRAME.Data[2]=8;LOW_CBROAD_FRAME.Data[3]=9;
  memcpy(&LOW_CBROAD_FRAME.Data[0], &usb_dpkg_data.vx_set, sizeof(float)); // 拷贝X坐标到前4字节
  memcpy(&LOW_CBROAD_FRAME.Data[4], &usb_dpkg_data.vy_set, sizeof(float)); // 拷贝Y坐标到后4字节
  // printf("angle: %f       ",usb_dpkg_data.vx_set);
  // printf("speed: %f       ",usb_dpkg_data.vy_set);
  USER_CAN_TxMessage(&LOW_CBROAD_FRAME);
}

static void high_Cbroad_Can_Send()
{
  /* 使用 get_gimbal_point() 获取只读云台状态 */
  const gimbal_t *g = get_gimbal_point();
  memcpy(&HIGH_CBROAD_FRAME.Data[0], &g->gimbal_pos.yaw_relattive_pos, sizeof(float)); // 拷贝X坐标到前4字节
  memset(&HIGH_CBROAD_FRAME.Data[4], usb_dpkg_data.Mode, sizeof(uint8_t));

  USER_CAN_TxMessage(&HIGH_CBROAD_FRAME);
}
