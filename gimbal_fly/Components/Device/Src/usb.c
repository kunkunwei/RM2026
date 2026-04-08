#include "usbd_cdc_if.h"
#include "usb.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "user_lib.h"
Usb_receive_data_t  usb_recvive_data={.buffer = NULL,.len=0};
Usb_dpkg_data_t     usb_dpkg_data={.vx_set=0,.vy_set=0,.Mode=0,.isShoot=0,.User=0};
Usb_AutoAim_t       usb_autoAim_data = {.autoMod=0};
extern uint8_t Usart_Mode;

uint8_t calculate_crc8(const uint8_t *data, size_t len);
void usbReceive_autoAim_handle(const uint8_t *data);
void usbReceive_Ros_handle(const uint8_t *Buf);
/**
  * @brief  usb的DP拉低一段时间，需要在usb_init之前调用
  *
  *         @note
  *         重新自举
  *
  * @param  NULL
  */
void usbReset(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = USB_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(USB_DP_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(USB_DP_GPIO_Port,USB_DP_Pin,GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(USB_DP_GPIO_Port,USB_DP_Pin,GPIO_PIN_SET);
}

/**
  * @brief  把结构体的数据发到上位机
  *
  *         @note
  *         记得匹配数据格式
  *
  * @param  Usb_send_data_t*: 发送的结构体的指针
  * @retval USB_SEND_OK OR USB_SEND_FAIL
  */
Usb_send_state_e usbSendData(Usb_send_data_t* Usb_send_data){

		char buffer[50];
    sprintf(buffer,"%.2f,%.2f,%.2f,%.2f,%2d,%2d,%.2f,%.2f\n",\
                                  Usb_send_data->yaw,Usb_send_data->pitch,\
                                  Usb_send_data->vx,Usb_send_data->vy,\
                                  Usb_send_data->robot_hp,Usb_send_data->heat,\
                                  Usb_send_data->position_x,Usb_send_data->position_y);

    if(CDC_Transmit_FS((uint8_t*)buffer,strlen(buffer)) == USBD_OK){
        return USB_SEND_OK;
    }
    else 
    {
        return USB_SEND_FAIL;
    }
}

Usb_send_state_e usbDebug_buff(uint8_t* Buf, uint32_t Len){
	if(CDC_Transmit_FS(Buf,Len) == USBD_OK){
    return USB_SEND_OK;
  }
  else 
  {
    return USB_SEND_FAIL;
  }
}

Usb_send_state_e usbDebug_float(float data){
    char buffer[15];
    sprintf(buffer, "%.2f\n",data);

    if(CDC_Transmit_FS((uint8_t*)buffer,strlen(buffer)) == USBD_OK){
        return USB_SEND_OK;
    }
    else 
    {
        return USB_SEND_FAIL;
    }
}

Usb_send_state_e usbDebug_uint(uint16_t data){
    char buffer[15];
    sprintf(buffer, "%d\r\n",data);

    if(CDC_Transmit_FS((uint8_t*)buffer,strlen(buffer)) == USBD_OK){
        return USB_SEND_OK;
    }
    else 
    {
        return USB_SEND_FAIL;
    }
}
//指向接收区

void cnm(uint8_t* Buf, uint32_t *Len){
    printf("cccccccccccc  %d\r\n",*Len);
}
void usbReceiveData(uint8_t* Buf, uint32_t *Len){

    usb_recvive_data.buffer = Buf;
    usb_recvive_data.len = *Len;

    if(usb_recvive_data.buffer == NULL)
      return ;
    if(usb_recvive_data.len != 12 && usb_recvive_data.len != 10){
        // printf("rec_  ! !%d\r\n",*Len);
        // for(int i = 0; i < *Len; i++){
        //     printf("%d ",Buf[i]);
        // }
        // printf("\r\n");
        return ;
    }
    
    //cnm(Buf,Len);
    if(usb_recvive_data.len == 10)
        usbReceive_autoAim_handle(Buf);
    else if(usb_recvive_data.len == 12)
        usbReceive_Ros_handle(Buf);
//     uint8_t crc,my_crc;
//     memcpy(&usb_dpkg_data.vx_set,Buf,sizeof(float));
//     memcpy(&usb_dpkg_data.vy_set,Buf+4,sizeof(float));
//     memcpy(&usb_dpkg_data.isShoot,Buf+8,sizeof(uint8_t));
//     memcpy(&usb_dpkg_data.Mode,Buf+9,sizeof(uint8_t));
//     memcpy(&usb_dpkg_data.User,Buf+10,sizeof(uint8_t));

//     memcpy(&crc,Buf+11,sizeof(uint8_t));

//     my_crc = calculate_crc8(Buf,11);
//    if(my_crc != crc){
//         printf("%d,%d\r\n",(int)my_crc,(int)crc);
//      return;
//    }
}

void limitSpeed(float* spd,float MAX){
  if( *spd > MAX)
    *spd = MAX;
  if( *spd < -MAX)
    *spd = -MAX;
}
// //解包


uint8_t calculate_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;      // 初始化 CRC 为 0x00
    uint8_t poly = 0x07;     // 使用多项式 0x07

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];      // 将输入字节与当前 CRC 异或
        for (uint8_t j = 0; j < 8; j++) { // 遍历每一位
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;  // 高位为1，移位后与多项式异或
            } else {
                crc <<= 1;    // 高位为0，仅左移
            }
        }
    }
    return crc;               // 返回最终的 CRC
}
/**
  * @brief  获取接收区指针
  *
  *         @note
  *         包括buff的头指针和大小len，原始数据，需要自己解析
  *
  * @param  NULL
  * @retval point of Usb_receive_data_t
  */

void usbReceive_autoAim_handle(const uint8_t *data){
    uint8_t crc,my_crc;
    memcpy(&usb_autoAim_data.minipc_target_yaw,     data,sizeof(float));
    memcpy(&usb_autoAim_data.minipc_target_pitch,   data+4,sizeof(float));
    memcpy(&usb_autoAim_data.User,                  data+8,sizeof(uint8_t));
    memcpy(&crc,                                    data+9,sizeof(uint8_t));

    my_crc = calculate_crc8(data,9);
    if(my_crc != crc){
         printf("crc_err ; %d,%d\r\n",my_crc,crc);
      return;
    }
    usb_autoAim_data.minipc_target_yaw = rad_format(usb_autoAim_data.minipc_target_yaw);
    usb_autoAim_data.minipc_target_pitch = rad_format(usb_autoAim_data.minipc_target_pitch);
    
    usb_autoAim_data.autoMod = (usb_autoAim_data.User >> 6) & 0x03;     //0000 0011;
    usb_autoAim_data.shoot_flag = (usb_autoAim_data.User >> 5) &0x01;   //0000 0001
    //printf("mod :%d , shoot:%d\r\n",usb_autoAim_data.autoMod,usb_autoAim_data.shoot_flag);
    Usart_Mode = usb_autoAim_data.autoMod;
    //printf("yaw:%f,pitch:%f,user:%d\r\n",usb_autoAim_data.minipc_target_yaw,usb_autoAim_data.minipc_target_pitch,usb_autoAim_data.User);
}

void usbReceive_Ros_handle(const uint8_t *Buf){
    uint8_t crc,my_crc;
    //printf("ROs\r\n");
    memcpy(&usb_dpkg_data.vx_set,Buf,sizeof(float));
    memcpy(&usb_dpkg_data.vy_set,Buf+4,sizeof(float));
    memcpy(&usb_dpkg_data.isShoot,Buf+8,sizeof(uint8_t));
    memcpy(&usb_dpkg_data.Mode,Buf+9,sizeof(uint8_t));
    memcpy(&usb_dpkg_data.User,Buf+10,sizeof(uint8_t));

    memcpy(&crc,Buf+11,sizeof(uint8_t));

    my_crc = calculate_crc8(Buf,11);
   if(my_crc != crc){
        printf("%d,%d\r\n",(int)my_crc,(int)crc);
     return;
   }
}
//Usb_receive_data_t* get_usb_receive_data(void){
//    return &usb_recvive_data;
//}
const Usb_dpkg_data_t*    getUsbDpkgData(void){
		return &usb_dpkg_data;
}

const Usb_AutoAim_t*    getUsbMiniPcPtr(void){
    return &usb_autoAim_data;
}