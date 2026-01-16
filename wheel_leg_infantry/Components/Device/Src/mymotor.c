#include "mymotor.h"

lk9025_motor_measure_t motor_right, motor_left;
dm8009_motor_measure_t motor_joint[4];
dji_motor_measure_t yaw_motor;  // Yaw电机

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
/**
 * @brief  DJI电机通用数据解析函数
 * @param  ptr        DJI电机数据结构体指针
 * @param  rx_message CAN接收数据缓冲区(8字节)
 * @retval None
 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    /* 解析DJI电机标准反馈数据 */
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);       // 编码器位置(0-8191)
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);        // 转速(rpm)
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]);    // 电流(mA)
    (ptr)->temp = rx_message[6];                                        // 温度(°C)

    // (ptr)->real_w = (ptr)->rpm * 2.0f * 3.14f / 60.0f;               // 角速度计算(rad/s)
    // 角速度计算(已注释): ptr->real_w = ptr->rpm*2*3.14/(60.0f * 36.0f);   // 2006角速度计算(rad/s)
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
/**
 * @brief  获取Yaw轴达妙电机数据指针
 * @retval dm_motor_measure_t* Yaw轴达妙电机数据结构体指针
 */
dji_motor_measure_t* get_yaw_motor(){
    return &yaw_motor;
}