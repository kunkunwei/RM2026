#include "main.h"
#include "can_task.h"
#include "old_pid.h"
#include "Gimbal_task.h"
#include "stepper_can.h"

static void Dji_Motor_Shoot_Can_Send(int16_t current_Shoot_Left, int16_t current_Shoot_Right, int16_t current_Shoot_Pull);


extern CAN_TxFrameTypeDef ALLShootTxFrame;

extern CAN_TxFrameTypeDef YAW_MOTOR_FRAME;
extern CAN_TxFrameTypeDef PITCH_MOTOR_FRAME;


void Can_Task(void const *argument)
{
    /* USER CODE BEGIN Can_Task */
    /* Infinite loop */
    extern gimbal_t gimbal;
    TickType_t systick = 0;
    // 状态查询计数器
    static uint32_t query_counter = 0;

    osDelay(3000);
    StepperCAN_Enable( &yaw_motor_Info,ENABLE);
    vTaskDelay(2);
    StepperCAN_Enable( &pitch_motor_Info,ENABLE);
    for (;;) {
        systick = xTaskGetTickCount();
        // 处理CAN发送队列
        CAN_Process_Send_Queue();

        // 摩擦轮控制逻辑
        if (systick <= 4000) {
            // 初始化阶段
            Dji_Motor_Shoot_Can_Send(0, 0, 0);

            // 步进电机归零
            StepperCAN_SetPositionAngle(&yaw_motor_Info, 0, 0, 0, 1);
            StepperCAN_SetPositionAngle(&pitch_motor_Info, 0, 0, 0, 1);
        } else {
            // 正常运行
            // 大疆电机控制（最高优先级）
            Dji_Motor_Shoot_Can_Send(gimbal.gimbal_shoot.shoot_motor_right->target_current,
                                     -gimbal.gimbal_shoot.shoot_motor_left->target_current,
                                     gimbal.gimbal_shoot.pull_motor->target_current);

            // 步进电机控制（高优先级）
            StepperCAN_SetPositionAngle(&yaw_motor_Info,
                gimbal.gimbal_pos.yaw_target_pos,
                gimbal.gimbal_pos.yaw_motor_measure->target_speed, 10, 1);

            StepperCAN_SetPositionAngle(&pitch_motor_Info,
                gimbal.gimbal_pos.pitch_target_pos,
                gimbal.gimbal_pos.pitch_motor_measure->target_speed, 10, 1);

            //定时查询步进电机状态（低优先级，每10个周期查一次）
            query_counter++;
            if (query_counter % 10 == 0) {
                StepperCAN_ReadSystemStatus(CAN1_YAW_MOTOR_ID);
                StepperCAN_ReadSystemStatus(CAN1_PITCH_MOTOR_ID);
            }
        }

        // 处理队列中剩余的帧（如果有）
        if (CAN_Get_Queue_Size() > 0) {
            CAN_Process_Send_Queue();
        }

        // 固定2ms周期
        osDelayUntil(&systick, 2);
    }
        // // 摩擦轮加拨蛋
        // if (systick <= 4000) {
        //     Dji_Motor_Shoot_Can_Send(0,
        //                              0,
        //                              0);
        //
        //     StepperCAN_SetPositionAngle(&yaw_motor_Info,0, 0, 0, 1);
        //     vTaskDelay(1);
        //     StepperCAN_SetPositionAngle(&pitch_motor_Info,0, 0, 0, 1);
        //     vTaskDelay(1);
        // } else {
        //     // 摩擦轮加拨蛋加pitch
        //     Dji_Motor_Shoot_Can_Send(gimbal.gimbal_shoot.shoot_motor_right->target_current,
        //                              -gimbal.gimbal_shoot.shoot_motor_left->target_current,
        //                              gimbal.gimbal_shoot.pull_motor->target_current);
        //
        //     StepperCAN_SetPositionAngle(&yaw_motor_Info,gimbal.gimbal_pos.yaw_target_pos, gimbal.gimbal_pos.yaw_motor_measure->target_speed, 10, 1);
        //     vTaskDelay(1);
        //     StepperCAN_SetPositionAngle(&pitch_motor_Info,gimbal.gimbal_pos.pitch_target_pos, gimbal.gimbal_pos.pitch_motor_measure->target_speed, 10, 1);
        //     vTaskDelay(1);
        //     osDelay(2);
        // }
        // LOW frequency

    /* USER CODE END Can_Task */
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
    // 使用简洁版本发送（高优先级）
    CAN_Send_Frame_Priority(&ALLShootTxFrame, CAN_PRIORITY_HIGH);
    // USER_CAN_TxMessage(&ALLShootTxFrame);
}
