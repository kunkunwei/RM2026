#include "mymotor.h"

lk9025_motor_measure_t motor_right, motor_left;
dm8009_motor_measure_t motor_joint[4];

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

void get_dm8009_motor_measure(dm8009_motor_measure_t* ptr, uint8_t *rx_message)
{
    ptr->err = rx_message[0]>>4;
    uint16_t tmp_pos = (int16_t) (((uint16_t)(rx_message[1]) <<8) | ((uint16_t)(rx_message[2])));
    ptr->pos = uint_to_float(tmp_pos,-12.5,12.5,16);

    uint16_t tmp_spd = (rx_message[3]<<4 | (rx_message[4]>>4) );
    ptr->speed = uint_to_float(tmp_spd,-45,45,12);

    uint16_t tmp_tor=((rx_message[4]&0xF)<<8)| rx_message[5];
    ptr->tor = uint_to_float(tmp_tor,-20,20,12);

    ptr->tmos_tmper = rx_message[6];
    ptr->coil_tmper = rx_message[7];

    //printf("%d,%.2f\r\n",rx_message[0]&0x0f,ptr->pos);s
}

//lk9025电机数据读取
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, uint8_t *rx_message)
{
    ptr->tmper = (int8_t)rx_message[1];
    ptr->tor_current = (int16_t)(rx_message[2] | rx_message[3] << 8);
    ptr->speed = (int16_t)(rx_message[4] | rx_message[5] << 8);
    ptr->pos = (uint16_t)(rx_message[6] | rx_message[7] << 8);
}

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void)
{
    return &motor_right;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void)
{
    return &motor_left;
}
//返回关节电机变量地址，通过指针方式获取原始数据
const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i)
{
    return &motor_joint[(i & 0x03)];
}