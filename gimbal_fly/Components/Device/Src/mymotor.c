/**
  ******************************************************************************
  * @file           : mymotor.c  
  * @brief          : 自定义电机接口实现文件
  * @author         : [作者名]
  * @date           : 2025-09-23
  ******************************************************************************
  * @attention      : 实现DJI电机、达妙电机、廉科电机的数据解析和控制接口
  *                  提供统一的电机数据访问方式，便于上层应用调用
  ******************************************************************************
  */

#include "mymotor.h"

/* 发射机构电机实例定义 */
dji_motor_measure_t shoot_motor_left,shoot_motor_right,pull_motor;  // 左右摩擦轮 + 拨弹电机

/* 达妙电机实例定义 */
dm_motor_measure_t yaw_motor = {.update_time = 0};                  // Yaw轴达妙电机，初始化时间戳
dm_motor_measure_t pitch_motor= {.update_time = 0};                 // Pitch轴电机
dm_motor_measure_t roll_motor= {.update_time = 0};                 // Roll轴电机


/* 私有函数声明 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);
/**
 * @brief  达妙DM8009电机数据解析函数
 * @param  ptr        达妙电机数据结构体指针
 * @param  rx_message CAN接收数据缓冲区(8字节)
 * @retval None
 */
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message)
{
    /* 解析错误状态码(高4位) */
    ptr->err = rx_message[0]>>4 & 0x0F;
    
    /* 解析位置数据(16位) */
    uint16_t tmp_pos = (int16_t) (((uint16_t)(rx_message[1]) <<8) | ((uint16_t)(rx_message[2])));
    ptr->last_yaw_pos =  ptr->pos;  // 保存上次位置
    ptr->pos = uint_to_float(tmp_pos,-3.141593f,3.141593f,16);  // 转换为弧度值(-π~π)
    
    /* 计算位置变化量并处理角度溢出 */
    ptr->deta_yaw_pos = ptr->pos - ptr->last_yaw_pos;
    if(ptr->deta_yaw_pos > 2.0f*3.14f/3.0f){
        ptr->deta_yaw_pos -= 2.0f*3.1415f;  // 正向溢出处理
    }
    else if(ptr->deta_yaw_pos < -2.0f*3.14f/3.0f){
        ptr->deta_yaw_pos += 2.0f*3.1415f;  // 负向溢出处理
    }

    /* 解析速度数据(12位) */
    uint16_t tmp_spd = (rx_message[3]<<4 | (rx_message[4]>>4) );
    ptr->speed = uint_to_float(tmp_spd,-30,30,12);  // 转换为rad/s(-30~30)

    /* 解析扭矩数据(12位) */
    uint16_t tmp_tor=((rx_message[4]&0xF)<<8)| rx_message[5];
    ptr->tor = uint_to_float(tmp_tor,-10,10,12);    // 转换为N·m(-10~10)

    /* 解析温度数据 */
    ptr->tmos_tmper = rx_message[6];  // MOS管温度
    ptr->coil_tmper = rx_message[7];  // 线圈温度

    /* 更新时间戳 */
    ptr->update_time = xTaskGetTickCount();
}


/**
 * @brief  无符号整型转换为浮点型函数
 * @param  X_int 输入的无符号整型数值
 * @param  X_min 最小浮点值
 * @param  X_max 最大浮点值
 * @param  Bits  数据位数
 * @retval float 转换后的浮点值
 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
    /// 根据范围和位数将无符号整型转换为浮点型 ///
    float span = X_max - X_min;      // 计算数值范围
    float offset = X_min;            // 偏移量
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

///////////////////////////DJI电机数据解析////////////////////////////////////
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

    // 角速度计算(已注释): ptr->real_w = ptr->rpm*2*3.14/(60.0f * 36.0f);
}


////////////////////////////////电机实例访问接口////////////////////////////////
/**
 * @brief  获取左摩擦轮电机数据指针
 * @retval dji_motor_measure_t* 左摩擦轮电机数据结构体指针
 */
dji_motor_measure_t* get_shoot_motor_left(){
    return &shoot_motor_left;
}

/**
 * @brief  获取右摩擦轮电机数据指针
 * @retval dji_motor_measure_t* 右摩擦轮电机数据结构体指针
 */
dji_motor_measure_t* get_shoot_motor_right(){
    return &shoot_motor_right;
}

/**
 * @brief  获取拨弹电机数据指针
 * @retval dji_motor_measure_t* 拨弹电机数据结构体指针
 */
dji_motor_measure_t* get_shoot_motor_pull(){
    return &pull_motor;
}


/**
 * @brief  获取Pitch轴电机数据指针
 * @retval dm_motor_measure_t* Pitch轴电机数据结构体指针
 */
dm_motor_measure_t* get_pitch_motor(){
    return &pitch_motor;
}

/**
 * @brief  获取Yaw轴达妙电机数据指针
 * @retval dm_motor_measure_t* Yaw轴达妙电机数据结构体指针
 */
dm_motor_measure_t* get_yaw_motor(){
    return &yaw_motor;
}
/**
 * @brief  获取Roll轴达妙电机数据指针
 * @retval dm_motor_measure_t* Roll轴达妙电机数据结构体指针
 */
dm_motor_measure_t* get_roll_motor(){
    return &roll_motor;
}
