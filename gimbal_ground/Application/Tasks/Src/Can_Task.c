#include "main.h"
#include "can_task.h"
#include "old_pid.h"
#include "Gimbal_task.h"
#include "usb.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "can.h"
#include "User_Task.h"
#include "mymotor.h" /* 显式引入，不再依赖 bsp_can.h 间接引入 */
// #define DEBUG

/* CAN Manager Instance */

void Damiao_Motor_Enable();
static void Damiao_Motor_CAN_Send(uint8_t Motor_ID, float Postion, float Velocity, float KP, float KD, float Torque);
static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left, int16_t current_Shoot_Right, int16_t current_Shoot_Pull);
static void Dji_Motor_Pull_Can_Send(int16_t current_Shoot_Pull);
static void Dji_Motor_Yaw_Can_Send(int16_t current_yaw);
// static void Chassis_Ctrl_Can_Send(Chassis_RC_Info_t *ctrl_info);

extern CAN_TxFrameTypeDef ALLShootTxFrame;

extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;
extern CAN_TxFrameTypeDef Chassis_Ctrl_FRAME_1;


void Can_Task(void const *argument)
{
    /* USER CODE BEGIN Can_Task */
    /* Infinite loop */
    TickType_t systick = 0;

    /* 等待云台初始化完成后再注册回调，避免访问空指针 */
    while (!is_gimbal_init_done()) osDelay(1);

    /* 将电机 CAN 接收回调注册到 BSP 层 */
    mymotor_register_can_callbacks();

    const gimbal_t *local_gimbal = get_gimbal_point(); /* 地址固定，循环外初始化一次即可 */

    Damiao_Motor_Enable();

    for (;;)
    {
        systick = xTaskGetTickCount();
        // Damiao_Motor_Enable();

        // 摩擦轮加拨蛋
        if (systick <= 1000)
        {
            Dji_Motor_Shoot_Can_Send(0,
                                     0,
                                     0);

            Damiao_Motor_CAN_Send(2, 0, 0, 0, 1.2f, 0);
            Dji_Motor_Yaw_Can_Send(0);
        }
        else
        {
            // 摩擦轮加拨蛋加pitch
            // CAN1
            Dji_Motor_Shoot_Can_Send(local_gimbal->gimbal_shoot.shoot_motor_right->target_current,
                                     -local_gimbal->gimbal_shoot.shoot_motor_left->target_current,
                                     local_gimbal->gimbal_shoot.pull_motor->target_current);
            // Dji_Motor_Shoot_Can_Send(0,
            //                          0,
            //                          2000);
            // 发出的tor>0，则枪管向下压，tor<0则枪管向上抬
            Damiao_Motor_CAN_Send(2, 0, 0, 0, 1.2f, local_gimbal->gimbal_pos.pitch_motor_measure->target_tor);
            // CAN2
            Dji_Motor_Yaw_Can_Send(local_gimbal->gimbal_pos.yaw_motor_measure->target_current);
            // osDelay(0);
            // Dji_Motor_Shoot_Can_Send(local_gimbal->gimbal_shoot.shoot_motor_right->target_current,
            //                         -local_gimbal->gimbal_shoot.shoot_motor_left->target_current,
            //                         local_gimbal->gimbal_shoot.pull_motor->target_current);
            // osDelay(1);
            // Dji_Motor_Pull_Can_Send(local_gimbal->gimbal_shoot.pull_motor->target_current);

            // vTaskDelay(1);


        }
        // LOW frequency
        // vTaskDelay(2);
        osDelayUntil(&systick, 1); // 1ms周期控制
    }
    /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID, float Postion, float Velocity, float KP, float KD, float Torque)
{
    static uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;
    CAN_TxFrameTypeDef *Motor_Frame = NULL;

    if (Motor_ID == 2)
    {
        Motor_Frame = &PITCH_MOTOR_FRAME;
    }

    if (Motor_Frame == NULL)
        return;

    Postion_Tmp = float_to_uint(Postion, -3.141593f, 3.141593f, 16);
    Velocity_Tmp = float_to_uint(Velocity, -30, 30, 12);
    KP_Tmp = float_to_uint(KP, 0, 500, 12);
    KD_Tmp = float_to_uint(KD, 0, 5, 12);
    Torque_Tmp = float_to_uint(Torque, -10, 10, 12);
    Motor_Frame->Data[0] = (uint8_t)(Postion_Tmp >> 8);
    Motor_Frame->Data[1] = (uint8_t)(Postion_Tmp);
    Motor_Frame->Data[2] = (uint8_t)(Velocity_Tmp >> 4);
    Motor_Frame->Data[3] = (uint8_t)((Velocity_Tmp & 0x0F) << 4) | (uint8_t)(KP_Tmp >> 8);
    Motor_Frame->Data[4] = (uint8_t)(KP_Tmp);
    Motor_Frame->Data[5] = (uint8_t)(KD_Tmp >> 4);
    Motor_Frame->Data[6] = (uint8_t)((KD_Tmp & 0x0F) << 4) | (uint8_t)(Torque_Tmp >> 8);
    Motor_Frame->Data[7] = (uint8_t)(Torque_Tmp);
    USER_CAN_TxMessage(Motor_Frame);
}
void Damiao_Motor_Enable()
{


   PITCH_MOTOR_FRAME.Data[0] = 0xFF;
   PITCH_MOTOR_FRAME.Data[1] = 0xFF;
   PITCH_MOTOR_FRAME.Data[2] = 0xFF;
   PITCH_MOTOR_FRAME.Data[3] = 0xFF;
   PITCH_MOTOR_FRAME.Data[4] = 0xFF;
   PITCH_MOTOR_FRAME.Data[5] = 0xFF;
   PITCH_MOTOR_FRAME.Data[6] = 0xFF;
   PITCH_MOTOR_FRAME.Data[7] = 0xFC;

    USER_CAN_TxMessage(&PITCH_MOTOR_FRAME);
}

void Damiao_Motor_Pitch_ClearErr()
{

    PITCH_MOTOR_FRAME.Data[0] = 0xFF;
    PITCH_MOTOR_FRAME.Data[1] = 0xFF;
    PITCH_MOTOR_FRAME.Data[2] = 0xFF;
    PITCH_MOTOR_FRAME.Data[3] = 0xFF;
    PITCH_MOTOR_FRAME.Data[4] = 0xFF;
    PITCH_MOTOR_FRAME.Data[5] = 0xFF;
    PITCH_MOTOR_FRAME.Data[6] = 0xFF;
    PITCH_MOTOR_FRAME.Data[7] = 0xFB;
    USER_CAN_TxMessage(&PITCH_MOTOR_FRAME);
}

static void Dji_Motor_Yaw_Can_Send(int16_t current_yaw)
{
    YAW_MOTOR_FRAME.Data[0] = current_yaw >> 8;
    YAW_MOTOR_FRAME.Data[1] = current_yaw;
    YAW_MOTOR_FRAME.Data[2] = 0;
    YAW_MOTOR_FRAME.Data[3] = 0;
    YAW_MOTOR_FRAME.Data[4] = 0;
    YAW_MOTOR_FRAME.Data[5] = 0;
    YAW_MOTOR_FRAME.Data[6] = 0;
    YAW_MOTOR_FRAME.Data[7] = 0;
    USER_CAN_TxMessage(&YAW_MOTOR_FRAME);
}

static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Right, int16_t current_Shoot_Left, int16_t current_Shoot_Pull)
{
    ALLShootTxFrame.Data[0] = current_Shoot_Right >> 8;
    ALLShootTxFrame.Data[1] = current_Shoot_Right;
    ALLShootTxFrame.Data[2] = current_Shoot_Left >> 8;
    ALLShootTxFrame.Data[3] = current_Shoot_Left;
    ALLShootTxFrame.Data[4] = current_Shoot_Pull >> 8;
    ALLShootTxFrame.Data[5] = current_Shoot_Pull;
    ALLShootTxFrame.Data[6] = 0;
    ALLShootTxFrame.Data[7] = 0;
    USER_CAN_TxMessage(&ALLShootTxFrame);
}
// static void Dji_Motor_Pull_Can_Send(int16_t current_Shoot_Pull)
// {
//     Shoot_Pull_TxFrame.Data[0] = 0;
//     Shoot_Pull_TxFrame.Data[1] = 0;
//     Shoot_Pull_TxFrame.Data[2] = 0;
//     Shoot_Pull_TxFrame.Data[3] = 0;
//     Shoot_Pull_TxFrame.Data[4] = current_Shoot_Pull >> 8;
//     Shoot_Pull_TxFrame.Data[5] = current_Shoot_Pull;
//     Shoot_Pull_TxFrame.Data[6] = 0;
//     Shoot_Pull_TxFrame.Data[7] = 0;
//     USER_CAN_TxMessage(&Shoot_Pull_TxFrame);
// }
