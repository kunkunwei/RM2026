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
#include "bsp_can.h"
#include "vofa.h"

/* 发射机构电机实例定义（static：只有本文件的 CAN 回调可写；外部通过 getter 只读） */
static dji_motor_measure_t shoot_motor[4], pull_motor;  // 左右摩擦轮 + 拨弹电机
static dji_motor_measure_t chassis_motor[4];  // 底盘四个电机
/* 达妙电机实例定义 */
static dm_motor_measure_t yaw_motor= {.update_time = 0};   // Yaw电机
static dm_motor_measure_t pitch_motor = {.update_time = 0};                  // Pitch轴电机



/* 私有函数声明 */

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
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief  达妙电机数据解析函数
 * @param  ptr        达妙电机数据结构体指针
 * @param  rx_message CAN接收数据缓冲区(8字节)
 * @retval None
 */
void get_dm_motor_measure(dm_motor_measure_t* ptr, uint8_t *rx_message)
{
    (ptr)->id = (uint8_t)(rx_message[0]& 0x0f);
    (ptr)->err = (uint8_t)(rx_message[0] >> 4);
    (ptr)->pos_int = (uint16_t)(rx_message[1] << 8 | rx_message[2]);
    (ptr)->vel_int = (uint16_t)(rx_message[3] << 4 | rx_message[4] >> 4);
    (ptr)->tor_int = (uint16_t)((rx_message[4] & 0x0f) << 8 | rx_message[5]);
    (ptr)->T_Mos = (float)(rx_message[6]);
    (ptr)->T_Rotor = (float)(rx_message[7]);

    ptr->pos = uint_to_float((ptr)->pos_int,P_MIN,P_MAX,16);  // 转换为弧度值(-π~π)
    ptr->vel = uint_to_float((ptr)->vel_int,V_MIN,V_MAX,12);    // 转换为rad/s(-30~30)
    ptr->tor = uint_to_float((ptr)->tor_int,T_MIN,T_MAX,12);    // 转换为N·m(-10~10)
    /* 更新时间戳 */
    ptr->update_time = xTaskGetTickCount();
    // HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
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
    (ptr)->last_ecd = (ptr)->ecd;
    (ptr)->ecd = (uint16_t) (rx_message[0] << 8 | rx_message[1]);       // 编码器位置(0-8191)
    (ptr)->rpm = (int16_t)(rx_message[2] << 8 | rx_message[3]);        // 转速(rpm)
    (ptr)->current = (int16_t)(rx_message[4] << 8 | rx_message[5]);    // 电流(mA)
    (ptr)->temp = rx_message[6];                                        // 温度(°C)
}

////////////////////////////////电机实例访问接口////////////////////////////////
/**
 * @brief  获取摩擦轮电机数据指针
 * @retval dji_motor_measure_t* 摩擦轮电机数据结构体指针
 */
dji_motor_measure_t* get_shoot_motor(uint8_t i){
    return &shoot_motor[(i & 0x03)];
}

/**
 * @brief  获取拨弹电机数据指针
 * @retval dji_motor_measure_t* 拨弹电机数据结构体指针
 */
dji_motor_measure_t* get_shoot_motor_pull(){
    return &pull_motor;
}
/**
 * @brief  获取底盘电机数据指针
 * @param  i 电机编号(0-3)
 * @retval
 *
 * */

dji_motor_measure_t* get_chassis_motor(uint8_t i){
    return &chassis_motor[(i & 0x03)];
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

////////////////////////////////CAN回调注册////////////////////////////////
/**
 * @brief CAN1 接收中断分发回调
 * @note  将 CAN 原始帧路由到对应的电机解析函数，由 BSP 层调用
 */
static void CAN1_MotorRxDispatch(uint32_t stdId, uint8_t data[8])
{
    // uart_printf(&huart6, "CAN1 Rx: ID=0x%03X\r\n",stdId);
    switch (stdId)
    {
    case CAN1_SHOOT_PULL_MOTOR_ID:
        get_dji_motor_measure(&pull_motor, data);
        break;
    case CAN1_YAW_MOTOR_FB_ID:
        get_dm_motor_measure(&yaw_motor, data);
        break;
    case CAN1_CHASSIS_MOTOR_1_ID:
    case CAN1_CHASSIS_MOTOR_2_ID:
    case CAN1_CHASSIS_MOTOR_3_ID:
    case CAN1_CHASSIS_MOTOR_4_ID:
        get_dji_motor_measure(&chassis_motor[stdId-0x201], data);
        // uart_printf(&huart6, "CAN1 Rx: ID=0x%03X\r\n",stdId);
        break;
    default:
        break;
    }
}

/**
 * @brief CAN2 接收中断分发回调
 */
static void CAN2_MotorRxDispatch(uint32_t stdId, uint8_t data[8])
{
    // uart_printf(&huart6, "CAN2 Rx: ID=0x%03X\r\n",stdId);
    switch (stdId)
    {
    case CAN2_SHOOT_MOTOR_LEFT_1_ID:
    case CAN2_SHOOT_MOTOR_RIGHT_1_ID:
    case CAN2_SHOOT_MOTOR_LEFT_2_ID:
    case CAN2_SHOOT_MOTOR_RIGHT_2_ID:
        get_dji_motor_measure(&shoot_motor[stdId-0x201], data);
        break;
    case CAN2_PITCH_MOTOR_FB_ID:
        get_dm_motor_measure(&pitch_motor, data);
        break;
    default:
        break;
    }
}

/**
 * @brief 将电机 CAN 接收回调注册到 BSP 层
 * @note  应在 BSP_CAN_Init() 之后、任务启动之前调用
 */
void mymotor_register_can_callbacks(void)
{
    BSP_CAN1_RegisterRxCallback(CAN1_MotorRxDispatch);
    BSP_CAN2_RegisterRxCallback(CAN2_MotorRxDispatch);
}

