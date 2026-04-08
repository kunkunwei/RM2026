#ifndef __USB_H__
#define __USB_H__

#include "minipc.h"

/*
*   在此定义USB传输函数的返回值,失败返回0，成功返回1
*/
typedef enum {
    USB_SEND_FAIL = 0,
    USB_SEND_OK,
}Usb_send_state_e;
/*
*  接收原始数据的结构体
*/
typedef struct
{
	uint8_t* buffer;
	uint32_t len;
}Usb_receive_data_t;
//
typedef struct
{
	float vx_set;
    float vy_set;

    uint8_t isShoot;
    uint8_t Mode;
    uint8_t User;
    uint8_t crc;
}Usb_dpkg_data_t;

typedef struct
{
	float minipc_target_yaw;
    float minipc_target_pitch;
    float target_depth;
    uint8_t User;
    uint8_t crc;
    Control_Flag_t control_flag;

    bool shoot_flag;
    uint8_t autoMod;
}Usb_AutoAim_t;

//
typedef struct
{
	float wz_after_fliter;
	float vx_after_fliter;
}Usb_data_fliter_t;
/*
*   需要发送到上位机的数据
*/
typedef struct {  
    float vx;
    float vy;

    float pitch;
    float yaw;
    
    uint16_t robot_hp;
    float position_x;
    float position_y;
    uint16_t heat;
}Usb_send_data_t;

extern Usb_dpkg_data_t    usb_dpkg_data;
//usb复位，拉低DP一段时间，需要在配置成复用之前调用
void usbReset(void);

//发送数据
//通信
Usb_send_state_e usbSendData(Usb_send_data_t* Usb_send_data_t);
//dpkg
//void dpkg_receive_data(Usb_receive_data_t* receive_data);
//Debug用
Usb_send_state_e usbDebug_buff(uint8_t* Buf, uint32_t Len);
Usb_send_state_e usbDebug_float(float data);
Usb_send_state_e usbDebug_uint(uint16_t data);
// 获取接受区域指针
//Usb_receive_data_t* get_usb_receive_data(void);
const Usb_dpkg_data_t*    getUsbDpkgData(void);
const Usb_AutoAim_t*    getUsbMiniPcPtr(void);
//别管这个
void usbReceiveData(uint8_t* Buf, uint32_t *Len);
#endif 
