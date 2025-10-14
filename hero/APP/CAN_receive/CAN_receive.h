#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"
#include "DM4310.h"

#define CHASSIS_CAN CAN1
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x12,
    CAN_PIT_MOTOR_ID = 0x11,
		CAN_Fric1_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
		CAN_Fric2_MOTOR_ID = 0x208,
    CAN_GIMBAL_ALL_ID = 0x1FF,
		
		CAN_CHASSIS_POWER_ID = 0x123,
    // CAN_DM_MOTOR_ID = 0x000,	
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;           //转子机械角度
    int16_t speed_rpm;      //转子转速
    int16_t given_current;  //实际转矩电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;       //上次转子机械角度
} motor_measure_t;
typedef struct
{
    fp32 input_power;       //超电输入
    fp32 output_power;      //超电输出
		uint8_t electric_quantity; //电量
		uint8_t err;						//超电错误码
} power_measure_t;
extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送摩擦轮控制命令，其中rev为保留字节
extern void CAN_CMD_Fric(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//发送拨弹轮控制命令，其中rev为保留字节
extern void CAN_CMD_Stir(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_CMD_CHASSIS_Power(uint16_t limit_power,uint16_t rev1,uint16_t rev2,uint16_t rev3);
//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//返回yaw电机变量地址，通过指针方式获取原始数据
extern const DMMotor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const DMMotor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//返回fric1电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Fric1_Motor_Measure_Point(void);
//返回fric2电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Fric2_Motor_Measure_Point(void);

//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);
//获取底盘功率
extern void get_chassis_power(fp32 *power,fp32 *out);
// const DMMotor_measure_t *get_DM_Motor_Measure_Point(void);

//传入UI的全局变量
extern uint8_t cap_electric_quantity;
extern uint8_t shoot_vel_state;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_solve(void);
#endif

#endif
