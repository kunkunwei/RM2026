#include "usbd_cdc_if.h" // USB CDC接口头文件
#include "usb.h"         // USB相关头文件
#include "stdio.h"       // 标准输入输出
#include <stdlib.h>       // 标准库
#include <stdint.h>       // 标准整型
#include <string.h>       // 字符串处理
#include "main.h"        // 主头文件
#include "usart.h"
#include "vofa.h"

// USB接收数据结构体，初始化为NULL和0
Usb_receive_data_t usb_recvive_data={.buffer = NULL,.len=0};
// USB解包数据结构体，初始化为0
Usb_dpkg_data_t    usb_dpkg_data={.wz_set=0,.vx_set=0,.vy_set=0};

// 静态接收缓冲区
static uint8_t usb_rx_buffer[64];

// CRC8校验函数声明
uint8_t calculate_crc8(const uint8_t *data, size_t len);

/**
  * @brief  usb的DP拉低一段时间，需要在usb_init之前调用
  *         重新自举
  * @param  NULL
  */
void usbReset(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0}; // GPIO初始化结构体
    __HAL_RCC_GPIOA_CLK_ENABLE();           // 使能GPIOA时钟
    GPIO_InitStruct.Pin = USB_DP_Pin;       // 设置DP引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出模式
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // 不上拉不下拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速
    HAL_GPIO_Init(USB_DP_GPIO_Port, &GPIO_InitStruct); // 初始化GPIO
    HAL_GPIO_WritePin(USB_DP_GPIO_Port,USB_DP_Pin,GPIO_PIN_RESET); // 拉低DP
    HAL_Delay(100); // 延时100ms
    HAL_GPIO_WritePin(USB_DP_GPIO_Port,USB_DP_Pin,GPIO_PIN_SET);   // 拉高DP
}

/**
  * @brief  把结构体的数据发到上位机
  *         记得匹配数据格式
  * @param  Usb_send_data_t*: 发送的结构体的指针
  * @retval USB_SEND_OK OR USB_SEND_FAIL
  */
Usb_send_state_e usbSendData(Usb_send_data_t* Usb_send_data){
    char buffer[30];
    // 格式化发送数据到buffer
    sprintf(buffer,"%.2f,%.2f,%.2f,%.2f\n",\
            Usb_send_data->yaw,Usb_send_data->yaw_gyro,\
            Usb_send_data->v,Usb_send_data->kilometer);
    // 通过USB发送数据
    if(CDC_Transmit_FS((uint8_t*)buffer,strlen(buffer)) == USBD_OK){
        return USB_SEND_OK;
    }
    else 
    {
        return USB_SEND_FAIL;
    }
}

/**
  * @brief  发送调试缓冲区数据
  * @param  Buf: 数据缓冲区指针
  * @param  Len: 数据长度
  * @retval USB_SEND_OK OR USB_SEND_FAIL
  */
Usb_send_state_e usbDebug_buff(uint8_t* Buf, uint32_t Len){
    if(CDC_Transmit_FS(Buf,Len) == USBD_OK){
        return USB_SEND_OK;
    }
    else
    {
        return USB_SEND_FAIL;
    }
}

/**
  * @brief  发送单个float调试数据
  * @param  data: 浮点数
  * @retval USB_SEND_OK OR USB_SEND_FAIL
  */
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

/**
  * @brief  发送单个uint16调试数据
  * @param  data: 无符号16位整型
  * @retval USB_SEND_OK OR USB_SEND_FAIL
  */
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

// 指向接收区 float 接收函数
void usbReceiveData(uint8_t* Buf, const uint32_t *Len)
{
    usb_recvive_data.buffer = Buf;
    usb_recvive_data.len = *Len;

    if(usb_recvive_data.buffer == NULL || usb_recvive_data.len != 13)
        return;
    //printf("ROs\r\n");
    // 先计算CRC（不包括CRC字节本身）
    uint8_t my_crc = calculate_crc8(Buf, 12); // 只计算前12个字节
    uint8_t received_crc = Buf[12]; // 第13个字节是CRC
    if(my_crc != received_crc)
    {
        uint8_t *b={"defet"};
        uart_printf(&huart6,"crc_err ; %d",my_crc);
        usbDebug_buff(b,sizeof(b));

        return;
    }
    // CRC校验通过，解析数据
    memcpy(&usb_dpkg_data.vx_set, Buf, sizeof(float));
    memcpy(&usb_dpkg_data.vy_set, Buf+4, sizeof(float));
    memcpy(&usb_dpkg_data.wz_set, Buf+8, sizeof(float));
    // uart_printf(&huart1,"crc_suss ; %d,%d,%f,%f\r\n",my_crc,received_crc,usb_dpkg_data.vx_set,usb_dpkg_data.wz_set);
    // uart_printf(&huart6,"crc_suss_vx ; %d,%d,%d,%d\r\n",Buf[0],Buf[1],Buf[2],Buf[3]);
    // uart_printf(&huart6,"crc_suss_vy ; %d,%d,%d,%d\r\n",Buf[4],Buf[5],Buf[6],Buf[7]);
    // uart_printf(&huart6,"crc_suss_wz ; %d,%d,%d,%d,%d\r\n",Buf[8],Buf[9],Buf[10],Buf[11],Buf[12]);
    uart_printf(&huart6,"crc_suss ; %f,%f,%f\r\n",usb_dpkg_data.vx_set,usb_dpkg_data.vy_set,usb_dpkg_data.wz_set);
}
// void usbReceiveData(uint8_t* Buf, const uint32_t *Len)
// {
//     if(Buf == NULL || *Len != 9) return;
//
//     // CRC校验
//     uint8_t my_crc = calculate_crc8(Buf, 8);
//     if(my_crc != Buf[8])
//     {
//         uart_printf(&huart1, "crc_err ; %d\r\n", my_crc);
//         return;
//     }
//
//     // 将前8个字节转换为字符串
//     char data_str[9];
//     memcpy(data_str, Buf, 8);
//     data_str[8] = '\0';  // 添加字符串结束符
//
//     // 使用sscanf解析字符串格式的浮点数
//     float vx, wz;
//     if(sscanf(data_str, "%f %f", &vx, &wz) == 2) {
//         usb_dpkg_data.vx_set = vx;
//         usb_dpkg_data.wz_set = wz;
//         uart_printf(&huart1, "crc_suss ; %.2f, %.2f\r\n", vx, wz);
//     } else {
//         uart_printf(&huart1, "parse failed: %s\r\n", data_str);
//     }
// }
/**
  * @brief  限制速度范围
  * @param  spd: 速度指针
  * @param  MAX: 最大速度
  */
void limitSpeed(float* spd,float MAX){
    if( *spd > MAX)
        *spd = MAX;
    if( *spd < -MAX)
        *spd = -MAX;
}


/**
  * @brief  CRC8校验计算
  * @param  data: 数据指针
  * @param  len: 数据长度
  * @retval 计算得到的CRC8值
  */
uint8_t calculate_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    uint8_t poly = 0x07;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
  * @brief  获取接收区指针
  *         包括buff的头指针和大小len，原始数据，需要自己解析
  * @param  NULL
  * @retval point of Usb_receive_data_t
  */
Usb_receive_data_t* get_usb_receive_data(void){
    return &usb_recvive_data;
}

/**
  * @brief  获取解包数据结构体指针
  * @retval Usb_dpkg_data_t结构体指针
  */
Usb_dpkg_data_t*    getUsbDpkgData(void){
    return &usb_dpkg_data;
}

// /**
//   * @brief  数据解包函数
//   * @param  receive_data: 接收数据结构体指针
//   */
// void dpkg_receive_data(Usb_receive_data_t* receive_data){
//     char total;
//     int result = sscanf((const char*)receive_data->buffer, " %f,%f,%c",
//                         &usb_dpkg_data.wz_set,
//                         &usb_dpkg_data.vx_set,
//                         &total);
//     // 检查解析结果
//     if(result != 3){
//         usb_dpkg_data.vx_set=0.0f;
//         usb_dpkg_data.wz_set=0.0f;
//     }
// }
