//
// Created by kun on 2026/3/12.
//

#ifndef WHEEL_LEG_INFANTRY_SUPER_POWER_H
#define WHEEL_LEG_INFANTRY_SUPER_POWER_H
#include "main.h"

typedef struct
{
    fp32 input_power;       //超电输入
    fp32 output_power;      //超电输出
    uint8_t electric_quantity; //电量
    uint8_t err;
} power_measure_t;
extern power_measure_t chassis_super_power;
//解析底盘功率
extern void get_power_measure(power_measure_t *ptr,uint8_t *rx_message);
//获取底盘功率
extern void get_chassis_power(fp32 *power,fp32 *out,uint8_t *electric_quantity);
#endif //WHEEL_LEG_INFANTRY_SUPER_POWER_H