#include "Ros_Task.h"

#include "observe_task.h"
#include "usb.h"
#include "Gimbal_task.h"
Usb_send_data_t Usb_send_data;
const Usb_dpkg_data_t* Usb_receive_data = NULL;
Usb_data_fliter_t usb_fliter_data;

extern gimbal_t gimbal;

/////////

extern CAN_TxFrameTypeDef LOW_CBROAD_FRAME;
extern refree_info_t refree_info;
/////
void Ros_Task(void const * argument)
{
  osDelay(3200);
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */

  extern INS_Info_Typedef INS_Info;
 // const chassis_move_t* local_chassis = get_chassis_control_point();

  Usb_receive_data = getUsbDpkgData();

  //first_order_filter_type_t wz_filter_t = {.input = wz, .frame_period=0.01f, .out=wz_fliter, .num=num};
  //first_order_filter_type_t v_filter_t =  {.input = v,  .frame_period=0.01f, .out=v_fliter,  .num=num};

  //first_order_filter_init(&wz_filter_t,0.01f,&num);
  //first_order_filter_init(&v_filter_t ,0.01f,&num);

  for(;;)
  {


	  // usbSendData(&Usb_send_data);


    vTaskDelay(50);
  }
  /* USER CODE END Ros_Task */
}
