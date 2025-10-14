#include "Ros_Task.h"
#include "Chassis_Task.h"
#include "observe_task.h"
#include "usb.h"
#include "Gimbal_task.h"
Usb_send_data_t Usb_send_data;
const Usb_dpkg_data_t* Usb_receive_data = NULL;
Usb_data_fliter_t usb_fliter_data;

extern gimbal_t gimbal;

/////////
static void Low_Cbroad_Can_Send();
static void high_Cbroad_Can_Send();
static void yaokong_Cbroad_Can_Send();

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

    Usb_send_data.yaw = gimbal.gimbal_pos.yaw_absolute_pos;
    Usb_send_data.pitch = gimbal.gimbal_pos.pitch_absolute_pos;

    Usb_send_data.vx =0.1f;
    Usb_send_data.vy = 0.2f;
    
    Usb_send_data.robot_hp = refree_info.Hp_Percentage;
    Usb_send_data.position_x = 0;
    Usb_send_data.position_y = 0;
    Usb_send_data.heat = refree_info.Shoot_Heat_Percentage;

	  usbSendData(&Usb_send_data);
    //yaokong_Cbroad_Can_Send();
		high_Cbroad_Can_Send();
    vTaskDelay(1);
    Low_Cbroad_Can_Send(); 

    vTaskDelay(50);
  }
  /* USER CODE END Ros_Task */
}

static void Low_Cbroad_Can_Send(){

  /* 使用memcpy直接拷贝浮点数的内存布局 */
//LOW_CBROAD_FRAME.Data[0]=1;
//LOW_CBROAD_FRAME.Data[1]=2;LOW_CBROAD_FRAME.Data[2]=8;LOW_CBROAD_FRAME.Data[3]=9;
 memcpy(&LOW_CBROAD_FRAME.Data[0], &usb_dpkg_data.vx_set, sizeof(float)); // 拷贝X坐标到前4字节
 memcpy(&LOW_CBROAD_FRAME.Data[4], &usb_dpkg_data.vy_set, sizeof(float)); // 拷贝Y坐标到后4字节
// printf("angle: %f       ",usb_dpkg_data.vx_set);
// printf("speed: %f       ",usb_dpkg_data.vy_set);
USER_CAN_TxMessage(&LOW_CBROAD_FRAME);
}

static void high_Cbroad_Can_Send(){
  /* 使用memcpy直接拷贝浮点数的内存布局 */
  memcpy(&HIGH_CBROAD_FRAME.Data[0], &gimbal.gimbal_pos.yaw_relattive_pos, sizeof(float)); // 拷贝X坐标到前4字节
  //printf("xnm\r\n");
  memset(&HIGH_CBROAD_FRAME.Data[4],usb_dpkg_data.Mode,sizeof(uint8_t));

  USER_CAN_TxMessage(&HIGH_CBROAD_FRAME);
}
// uint16_t ch2,ch3,ch0;
// uint8_t rcs1;
// static void yaokong_Cbroad_Can_Send(){

// ch2=(uint16_t)remote_ctrl.rc.ch[2]+1024;
//   ch3=(uint16_t)remote_ctrl.rc.ch[3]+1024;
// ch0=(uint16_t)remote_ctrl.rc.ch[0]+1024;
// rcs1=(uint8_t)remote_ctrl.rc.s[1];

//   yaokong_CBROAD_FRAME.Data[0] = ch2 >> 8;
//   yaokong_CBROAD_FRAME.Data[1] = ch2;
//   //printf("111");'
// //	printf("ch2: %d       ",ch2);
// //	printf("ch3: %d       ",ch3);
// //		printf("ch0: %d       ",ch0);
//   yaokong_CBROAD_FRAME.Data[2] = ch3 >> 8;
//   yaokong_CBROAD_FRAME.Data[3] = ch3;
//   yaokong_CBROAD_FRAME.Data[4] = ch0 >> 8;
//   yaokong_CBROAD_FRAME.Data[5] = ch0;	
//   //printf("222");
// //	printf("rcs1: %d       ",rcs1);

//   //yaokong_CBROAD_FRAME.Data[0] = remote_ctrl.rc.s[1] ;
//   yaokong_CBROAD_FRAME.Data[6] = remote_ctrl.rc.s[1];
//   //printf("333");
// USER_CAN_TxMessage(&yaokong_CBROAD_FRAME);
// }