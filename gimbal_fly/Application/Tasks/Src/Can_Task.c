#include "main.h"
#include "can_task.h"
#include "old_pid.h"
#include "Gimbal_task.h"
#include "usb.h"
// #define DEBUG
void Damiao_Motor_Enable(uint8_t ID);
static void Damiao_Motor_CAN_Send(uint8_t Motor_ID, float Postion, float Velocity, float KP, float KD, float Torque);
static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left, int16_t current_Shoot_Right, int16_t current_Shoot_Pull);
// static void Dji_Motor_Pitch_Can_Send(int16_t current_pitch);

extern CAN_TxFrameTypeDef ALLShootTxFrame;

extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;
extern CAN_TxFrameTypeDef ROLL_MOTOR_FRAME;

void Can_Task(void const *argument)
{
    /* USER CODE BEGIN Can_Task */
    /* Infinite loop */
    // extern gimbal_t gimbal;
    const  gimbal_t* locaal_gimbal=get_gimbal_point();
    TickType_t systick = 0;
    osDelay(1000);
    Damiao_Motor_Enable(1);
    osDelay(1);
    Damiao_Motor_Enable(2);
    osDelay(1);
    Damiao_Motor_Enable(3);

    for (;;) {
        systick = xTaskGetTickCount();
        // 摩擦轮加拨蛋
        if (systick <= 1000) {
            Dji_Motor_Shoot_Can_Send(0,
                                     0,
                                     0);
            // Dji_Motor_Pitch_Can_Send(0);
            // vTaskDelay(1);
            Damiao_Motor_CAN_Send(1,0, 0, 0, 1.2f, 0);
            // vTaskDelay(1);
            Damiao_Motor_CAN_Send(2,0, 0, 0, 1.2f, 0);
            vTaskDelay(1);
            // Damiao_Motor_CAN_Send(3,0, 0, 0, 1.2f, 0);
            // vTaskDelay(1);
        } else {
            // 摩擦轮加拨蛋加pitch
            Dji_Motor_Shoot_Can_Send(locaal_gimbal->gimbal_shoot.shoot_motor_right->target_current,
                                     -locaal_gimbal->gimbal_shoot.shoot_motor_left->target_current,
                                     locaal_gimbal->gimbal_shoot.pull_motor->target_current);
            // Dji_Motor_Pitch_Can_Send(gimbal.gimbal_pos.pitch_motor_measure->target_current);

            // vTaskDelay(1);

            Damiao_Motor_CAN_Send(1,0, 0, 0, 1.2f, locaal_gimbal->gimbal_pos.yaw_motor_measure->target_tor);//电机倒置需要取反
            // vTaskDelay(1);
            Damiao_Motor_CAN_Send(2,0, 0, 0, 1.2f, locaal_gimbal->gimbal_pos.pitch_motor_measure->target_tor);
            vTaskDelay(0);
            // Damiao_Motor_CAN_Send(3,0, 0, 0, 1.2f, -locaal_gimbal->gimbal_pos.roll_motor_measure->target_tor);
            // vTaskDelay(1);
        }
        // LOW frequency
        osDelayUntil(&systick, 1); // 1ms周期控制
    }
    /* USER CODE END Can_Task */
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static void Damiao_Motor_CAN_Send(uint8_t Motor_ID, float Postion, float Velocity, float KP, float KD, float Torque)
{
    static uint16_t Postion_Tmp, Velocity_Tmp, Torque_Tmp, KP_Tmp, KD_Tmp;
    CAN_TxFrameTypeDef *Motor_Frame;
    if (Motor_ID == 1)
        Motor_Frame = &YAW_MOTOR_FRAME;
    else if (Motor_ID == 2)
        Motor_Frame = &PITCH_MOTOR_FRAME;
    else if (Motor_ID == 3)
        Motor_Frame = &ROLL_MOTOR_FRAME;

    Postion_Tmp             = float_to_uint(Postion, -3.141593f, 3.141593f, 16);
    Velocity_Tmp            = float_to_uint(Velocity, -30, 30, 12);
    KP_Tmp                  = float_to_uint(KP, 0, 500, 12);
    KD_Tmp                  = float_to_uint(KD, 0, 5, 12);
    Torque_Tmp              = float_to_uint(Torque, -10, 10, 12);
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
void Damiao_Motor_Enable(uint8_t ID)
{
    CAN_TxFrameTypeDef *Motor_Frame;
    if (ID == 1)
        Motor_Frame = &YAW_MOTOR_FRAME;
    else if (ID == 2)
        Motor_Frame = &PITCH_MOTOR_FRAME;
    else if (ID == 3)
        Motor_Frame = &ROLL_MOTOR_FRAME;
     Motor_Frame->Data[0] = 0xFF;
     Motor_Frame->Data[1] = 0xFF;
     Motor_Frame->Data[2] = 0xFF;
     Motor_Frame->Data[3] = 0xFF;
     Motor_Frame->Data[4] = 0xFF;
     Motor_Frame->Data[5] = 0xFF;
     Motor_Frame->Data[6] = 0xFF;
     Motor_Frame->Data[7] = 0xFC;

    USER_CAN_TxMessage(Motor_Frame);
}

void Damiao_Motor_ClearErr(uint8_t ID)
{

    YAW_MOTOR_FRAME.Data[0] = 0xFF;
    YAW_MOTOR_FRAME.Data[1] = 0xFF;
    YAW_MOTOR_FRAME.Data[2] = 0xFF;
    YAW_MOTOR_FRAME.Data[3] = 0xFF;
    YAW_MOTOR_FRAME.Data[4] = 0xFF;
    YAW_MOTOR_FRAME.Data[5] = 0xFF;
    YAW_MOTOR_FRAME.Data[6] = 0xFF;
    YAW_MOTOR_FRAME.Data[7] = 0xFB;
    // USER_CAN_TxMessage(&YAW_MOTOR_FRAME);
    if (ID==1)
        USER_CAN_TxMessage(&YAW_MOTOR_FRAME);
    else if (ID==2)
        USER_CAN_TxMessage(&PITCH_MOTOR_FRAME);
    else if (ID==3)
        USER_CAN_TxMessage(&ROLL_MOTOR_FRAME);
}

// static void Dji_Motor_Pitch_Can_Send(int16_t current_pitch)
// {
//     PITCH_MOTOR_FRAME.Data[0] = current_pitch >> 8;
//     PITCH_MOTOR_FRAME.Data[1] = current_pitch;
//     PITCH_MOTOR_FRAME.Data[2] = 0;
//     PITCH_MOTOR_FRAME.Data[3] = 0;
//     PITCH_MOTOR_FRAME.Data[4] = 0;
//     PITCH_MOTOR_FRAME.Data[5] = 0;
//     PITCH_MOTOR_FRAME.Data[6] = 0;
//     PITCH_MOTOR_FRAME.Data[7] = 0;
//     USER_CAN_TxMessage(&PITCH_MOTOR_FRAME);
// }

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

//