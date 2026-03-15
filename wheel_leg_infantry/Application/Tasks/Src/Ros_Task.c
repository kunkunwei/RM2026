 #include "Ros_Task.h"
#include "Chassis_Task.h"
#include "ctl_chassis.h"
#include "observe_task.h"
#include "usart.h"
#include "vofa.h"

 Usb_send_data_t Usb_send_data;
Usb_dpkg_data_t* Usb_receive_data = NULL;
Usb_data_fliter_t usb_fliter_data;
void Ros_Task(void const * argument)
{
  osDelay(500);
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */
  TickType_t systick = 0;
  extern INS_Info_Typedef INS_Info;
  const chassis_move_t* local_chassis = get_chassis_control_point();

  Usb_receive_data = getUsbDpkgData();

  float wz,wz_fliter,v,v_fliter;
  const float num = 0.2f;

  first_order_filter_type_t wz_filter_t = {.input = wz, .frame_period=0.01f, .out=wz_fliter, .num=num};
  first_order_filter_type_t v_filter_t =  {.input = v,  .frame_period=0.01f, .out=v_fliter,  .num=num};

  first_order_filter_init(&wz_filter_t,0.01f,&num);
  first_order_filter_init(&v_filter_t ,0.01f,&num);

  for(;;)
  {
    systick = osKernelSysTick();
    // Usb_send_data.v = get_body_Spd();
    // Usb_send_data.kilometer = local_chassis->kilometer;

    // Usb_send_data.yaw = local_chassis->chassis_yaw;
    // Usb_send_data.yaw_gyro = INS_Info.gyro[2];
	  // usbSendData(&Usb_send_data);
      // uart_printf(&huart1,"comm_ok:%d\r\n,rx_time:%.2f,tx_time:%.2f\r\n,chassis_yaw:%.2fmod:%d\r\n"
          // ,chassis_comm.comm_ok,chassis_comm.last_rx_time,chassis_comm.last_tx_time,
            // chassis_comm.feedback.chassis_yaw,chassis_comm.feedback.status_flags&STATUS_MODE_MASK);
    osDelayUntil(&systick, 50);

    // wz = Usb_receive_data->wz_set;
    // v =  Usb_receive_data->vx_set;
    // first_order_filter_cali(&wz_filter_t,wz);
    // first_order_filter_cali(&v_filter_t,v);

    // usb_fliter_data.vx_after_fliter = v_filter_t.out;
    // usb_fliter_data.wz_after_fliter = wz_filter_t.out;
    //usbDebug_uint(5);
    //printf("%.2f,%.2f\r\n",usb_receive_data->vx_set,usb_receive_data->wz_set);
    //osDelay(5);
  }
  /* USER CODE END Ros_Task */
}