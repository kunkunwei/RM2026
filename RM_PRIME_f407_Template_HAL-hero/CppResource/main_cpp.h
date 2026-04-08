#ifndef __MAIN_CPP_H
#define __MAIN_CPP_H

#include "main.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"


#define MAX_CAN_DEVICES 30


#ifdef __cplusplus
extern "C" {
#endif

void app_init_function(void);


#ifdef __cplusplus
}
#endif


#endif //__MAIN_CPP_H
