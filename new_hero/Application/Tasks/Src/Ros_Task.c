#include "Ros_Task.h"


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

/////
static void ros_send_gimbal_attitude(void);

void Ros_Task(void const *argument)
{
  /* 等待云台初始化完成，避免访问空指针 */
  while (!is_gimbal_init_done())
    osDelay(1);
    TickType_t systick = 0;
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */

  for (;;)
  {
    systick = osKernelSysTick();
    ros_send_gimbal_attitude();


    osDelayUntil(&systick, 10); // 10ms周期控制
    // vTaskDelay(50);
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

  MiniPC_Sendgimbal(gimbal->ins_info->yaw_angle,-gimbal->ins_info->pit_angle);
}
