#include "mymotor.h"

dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;
dji_motor_measure_t pitch_motor;
dm_motor_measure_t yaw_motor;
dji_motor_measure_t chassis_motor[4];

dji_motor_measure_t feiniao_yaw,feibiao_roll,feibiao_pull;
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
//DM8009
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

///////////////////////////DJI////////////////////////////////////
void get_dji_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    //(ptr)->last_ecd = (ptr)->ecd;                                                          
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);           
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);     
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]); 
    (ptr)->temp = rx_message[6];  

    //ptr->real_w = ptr->rpm*2*3.14/(60.0f * 36.0f);
}
void get_chassis_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    //(ptr)->last_ecd = (ptr)->ecd;                                                          
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);           
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);     
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]); 
    (ptr)->temp = rx_message[6];  

    ptr->real_w = ptr->rpm*2*3.14/(60.0f * 19.0f);
}
void get_pitch_motor_measure(dji_motor_measure_t* ptr, uint8_t *rx_message)
{
    //(ptr)->last_ecd = (ptr)->ecd;                                                          
    (ptr)->pos = (uint16_t) (rx_message[0] << 8 | rx_message[1]);           
    (ptr)->rpm = (uint16_t)(rx_message[2] << 8 | rx_message[3]);     
    (ptr)->current = (uint16_t)(rx_message[4] << 8 | rx_message[5]); 
    (ptr)->temp = rx_message[6];  

    ptr->real_w = ptr->rpm*2*3.14/(60.0f);   //rad/s


    ptr->real_pos = (((int)(ptr->pos) - 7460)/8192.0f)*2.0f*3.14159f;   //rad

    if(ptr->real_pos < -3.1415f){
        ptr->real_pos += 2.0f*3.14159f;
    }
}
//返回指针
dji_motor_measure_t* get_shoot_motor_left(){
    return &shoot_motor_left;
}
dji_motor_measure_t* get_shoot_motor_right(){
    return &shoot_motor_right;
}
dji_motor_measure_t* get_shoot_motor_pull(){
    return &pull_motor;
}

dji_motor_measure_t* get_chassis_motor(uint8_t i){
    return &chassis_motor[i];
}

dji_motor_measure_t* get_pitch_motor(){
    return &pitch_motor;
}
dm_motor_measure_t* get_yaw_motor(){
    return &yaw_motor;
}

dji_motor_measure_t* get_feibiao_yaw_motor(){
    return &feiniao_yaw;
}
dji_motor_measure_t* get_feibiao_rollmotor(){
    return &feibiao_roll;
}
dji_motor_measure_t* get_feibiao_pull_motor(){
    return &feibiao_pull;
}
//////////////////////////////////////////////////////////////////
// //返回yaw电机变量地址，通过指针方式获取原始数据
// const lk9025_motor_measure_t *get_Right_Wheel_Motor_Measure_Point(void)
// {
//     return &motor_right;
// }
// //返回pitch电机变量地址，通过指针方式获取原始数据
// const lk9025_motor_measure_t *get_Left_Wheel_Motor_Measure_Point(void)
// {
//     return &motor_left;
// }
// //返回关节电机变量地址，通过指针方式获取原始数据
// const dm8009_motor_measure_t *get_Joint_Motor_Measure_Point(uint8_t i)
// {
//     return &motor_joint[(i & 0x03)];
// }
////////////////////////////////////////////////////////////////////////////
//获取YAW电机数据
void get_yaw_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message)
{
    (ptr)->err = rx_message[0]>>4;
    uint16_t tmp_pos = (int16_t) (((uint16_t)(rx_message[1]) <<8) | ((uint16_t)(rx_message[2])));
    ptr->pos = uint_to_float(tmp_pos,-12.5,12.5,16);

    uint16_t tmp_spd = (rx_message[3]<<4 | (rx_message[4]>>4) );
    ptr->speed = uint_to_float(tmp_spd,-30,30,12);
}