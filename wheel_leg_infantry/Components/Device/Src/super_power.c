//
// Created by kun on 2026/3/12.
//

#include "super_power.h"


power_measure_t chassis_super_power;

//解析底盘功率
void get_power_measure(power_measure_t *ptr,uint8_t *rx_message)
{                                                                                                                                                       \
    (ptr)->input_power = 0.01*((uint16_t)(rx_message[1] << 8 | rx_message[0]));
    (ptr)->output_power = 0.01*((uint16_t)(rx_message[3] << 8 | rx_message[2]));
    (ptr)->electric_quantity = (rx_message[4]);
    (ptr)->err = (rx_message[5]);
}

//获取底盘功率和能量
void get_chassis_power(fp32 *power,fp32 *out,uint8_t *electric_quantity)
{
    *power = chassis_super_power.input_power;
    *out = chassis_super_power.output_power;
    *electric_quantity = chassis_super_power.electric_quantity;
}