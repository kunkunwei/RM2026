#include "mymotor.h"

dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;
dji_motor_measure_t pitch_motor;
dm_motor_measure_t gimbal_motor[2]; //0 云台yaw，1 云台roll
dji_motor_measure_t chassis_motor[4];

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
//DM8009
/**
 * @brief 解析大妙电机CAN数据
 */
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message)
{
    ptr->err = rx_message[0]>>4;
    uint16_t tmp_pos = (int16_t) (((uint16_t)(rx_message[1]) <<8) | ((uint16_t)(rx_message[2])));
    ptr->pos = uint_to_float(tmp_pos,-12.5,12.5,16);

    uint16_t tmp_spd = (rx_message[3]<<4 | (rx_message[4]>>4) );
    ptr->speed = uint_to_float(tmp_spd,-30,30,12);

    uint16_t tmp_tor=((rx_message[4]&0xF)<<8)| rx_message[5];
    ptr->tor = uint_to_float(tmp_tor,-10,10,12);

    ptr->tmos_tmper = rx_message[6];
    ptr->coil_tmper = rx_message[7];

    //printf("%d,%.2f\r\n",rx_message[0]&0x0f,ptr->pos);s
}

/**
 * @brief 读取LK9025电机数据
 */
void get_lk9025_motor_measure(lk9025_motor_measure_t* ptr, uint8_t *rx_message)
{
    ptr->tmper = (int8_t)rx_message[1];
    ptr->tor_current = (int16_t)(rx_message[2] | rx_message[3] << 8);
    ptr->speed = (int16_t)(rx_message[4] | rx_message[5] << 8);
    ptr->pos = (uint16_t)(rx_message[6] | rx_message[7] << 8);
}

/**
 * @brief 无符号整型转浮点型
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// 将无符号整型转换为浮点型，给定范围和位数 ///
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

///////////////////////////DJI////////////////////////////////////
/**
 * @brief 解析DJI电机CAN数据
 */
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    //(ptr)->last_ecd = (ptr)->ecd;                                                          
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);           
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);     
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]); 
    (ptr)->temp = rx_message[6];  

    //ptr->real_w = ptr->rpm*2*3.14/(60.0f * 36.0f);
}

/**
 * @brief 解析底盘电机CAN数据
 */
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    //(ptr)->last_ecd = (ptr)->ecd;                                                          
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);           
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);     
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]); 
    (ptr)->temp = rx_message[6];  

    ptr->real_w = ptr->rpm*2*3.14/(60.0f * 19.0f);
}
// void get_pitch_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
// {
//     //(ptr)->last_ecd = (ptr)->ecd;
//     (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);
//     (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);
//     (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]);
//     (ptr)->temp = rx_message[6];
//
//     ptr->real_w = ptr->rpm*2*3.14/(60.0f);   //rad/s
//
//
//     ptr->real_pos = (((int)(ptr->pos) - 7460)/8192.0f)*2.0f*3.14159f;   //rad
//
//     if(ptr->real_pos < -3.1415f){
//         ptr->real_pos += 2.0f*3.14159f;
//     }
// }

//返回指针
/**
 * @brief 获取左摩擦轮电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_left(){
    return &shoot_motor_left;
}
/**
 * @brief 获取右摩擦轮电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_right(){
    return &shoot_motor_right;
}
/**
 * @brief 获取拨蛋电机数据指针
 */
dji_motor_measure_t* get_shoot_motor_pull(){
    return &pull_motor;
}

/**
 * @brief 获取底盘电机数据指针
 * @param i 电机编号（0~3）
 */
dji_motor_measure_t* get_chassis_motor(uint8_t i){
    return &chassis_motor[i];
}

/**
 * @brief 获取云台电机数据指针
 * @param i 电机编号
 */
dm_motor_measure_t* get_gimbal_motor(uint8_t i){
    return &gimbal_motor[i];
}


//////////////////////////////////////////////////////////////////
