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



/* 私有函数声明 */
static float uint_to_float(int X_int, float X_min, float X_max, int Bits);


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




