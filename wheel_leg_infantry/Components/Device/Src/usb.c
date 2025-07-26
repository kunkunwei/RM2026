#include "usbd_cdc_if.h"
#include "usb.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
Usb_receive_data_t usb_recvive_data={.buffer = NULL,.len=0};
Usb_dpkg_data_t    usb_dpkg_data={.wz_set=0,.vx_set=0};

uint8_t calculate_crc8(const uint8_t *data, size_t len);

void dpkgReceiveData(Usb_receive_data_t* receive_data);
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

		char buffer[30];
    sprintf(buffer,"%.2f,%.2f,%.2f,%.2f\n",\
                                          Usb_send_data->yaw,Usb_send_data->yaw_gyro,\
                                          Usb_send_data->v,Usb_send_data->kilometer);

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
void usbReceiveData(uint8_t* Buf, uint32_t *Len){

    //printf("rec_%d\r\n",*Len);
    usb_recvive_data.buffer = Buf;
    usb_recvive_data.len = *Len;
    // usb_dpkg_data.vx_set=0.0f;
    // usb_dpkg_data.wz_set=0.0f;

    if(usb_recvive_data.buffer == NULL)
      return ;
    if(usb_recvive_data.len != 9)
      return ;

    uint8_t crc,my_crc;
    memcpy(&usb_dpkg_data.wz_set,Buf,sizeof(float));
    memcpy(&usb_dpkg_data.vx_set,Buf+4,sizeof(float));
    memcpy(&crc,Buf+8,sizeof(uint8_t));

    my_crc = calculate_crc8(Buf,8);
    //printf("%d\r\n",(int)my_crc);
    if(my_crc != crc){
      printf("crc error\r\n");
      usb_dpkg_data.vx_set=0.0f;
      usb_dpkg_data.wz_set=0.0f;
      return;
    }
    //printf(" data : %.2f,%.2f\r\n",usb_dpkg_data.wz_set,usb_dpkg_data.vx_set);
}
void limitSpeed(float* spd,float MAX){
  if( *spd > MAX)
    *spd = MAX;
  if( *spd < -MAX)
    *spd = -MAX;
}
// //解包
// void dpkgReceiveData(Usb_receive_data_t* receive_data){

//     char total;
// 		int result = sscanf(receive_data->buffer, " %f,%f,%c", 
//                         &usb_dpkg_data.wz_set, 
//                         &usb_dpkg_data.vx_set,
//                         &total);
//     if(total != 'm'){
//       usb_dpkg_data.vx_set=0.0f;
//       usb_dpkg_data.wz_set=0.0f;
//     }
//     //限幅
//     limitSpeed(&usb_dpkg_data.wz_set,2.0f);
//     limitSpeed(&usb_dpkg_data.vx_set,2.0f);
// }

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
//Usb_receive_data_t* get_usb_receive_data(void){
//    return &usb_recvive_data;
//}
Usb_dpkg_data_t*    getUsbDpkgData(void){
		return &usb_dpkg_data;
}