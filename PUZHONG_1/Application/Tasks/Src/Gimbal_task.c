#include "Gimbal_task.h"
#include "main.h"
#include "old_pid.h"
#include "stepper_pwm_control.h"
#include "tim.h"
#include "usart.h"

//
#define rc_deadline_limit(input, output, dealine)        \
{                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
}


void Gimbal_Task(void const * argument)
{
  /* Infinite loop */
    // uint16_t encoder=0 ;
    // int32_t Position=0 ;
    // bool u_flag=true;
    // Stepper_Init();
    uint8_t pwmch1[3200]; //定义内存缓存
    //一个周期是1us
    //100%占空比是1000

    volatile uint8_t pwm_lock = 0; //定义DMA发送完成标识。
    volatile uint32_t pwmflag[4]; //定义DMA发送完成标识数组。
    osDelay(800);
    TickType_t systick = 0;
    // Stepper_MoveRelative( 0, 1000, 25, 10.0);

    // Motor_SetSubdivision(EMM42_MOTOR_1_ADDR, 16);
    // Stepper_MoveRelative( 0, 100, 25, -100.0);
    // uint8_t i=1;
    // Stepper_MoveRelative( 0, 30, 25, -1.0f); // Move motor 0 (EMM42_MOTOR_1_ADDR) to 90 degrees at speed 100 with acceleration 25
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0); //设置PWM通道1的初始占空比为0
    // Stepper_SetSpeed( 0, 000, 25); // 设置电机0的速度为1000步/秒，25步/秒²的加速度
    Stepper_MoveRelative( 0, 1000, 25, -10.0); // Move motor 0 (EMM42_MOTOR_1_ADDR) to 90 degrees at speed 100 with acceleration 25
    for(;;)
    {
        Stepper_Update();

        // for (uint8_t i = 0; i < 32; i++)
        // // {
        //     for (uint8_t j = 0; j < 100; j++)
        //     {
        //         // pwmch1[j+100*i] = (j%2)?1000:0; //设置PWM通道1的占空比
        //         __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 1000);
        //     }
        //     // pwmch1[i] = (i&0x01)?900:500; //设置PWM通道1的占空比
        // // }
        // __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, 0);
        // pwmch1[3199] = 0; //最后一个值必须为0，表示DMA传输结束
        // pwm_lock=1; //设置DMA发送完成标识
        // pwmflag[0]=HAL_TIM_PWM_Start_DMA( &htim1, TIM_CHANNEL_1, (uint32_t*)pwmch1, 3200); //开始DMA传输
        // while (pwm_lock);
        // for (uint8_t i = 0; i < 12; i++)
        // {
        //     pwmch1[i] = (i&0x01)?500:900; //设置PWM通道1的占空比
        // }
        // pwmch1[11] = 0; //最后一个值必须为0，表示DMA传输结束
        // pwm_lock=1; //设置DMA发送完成标识
        // pwmflag[0]=HAL_TIM_PWM_Start_DMA( &htim1, TIM_CHANNEL_1, (uint32_t*)pwmch1, 128); //开始DMA传输
        // while (pwm_lock);
        // // Stepper_MoveRelative( 0, 1000, 25, 10.0);
        // Stepper_MoveRelative( 0, 100, 25, -1.0);

        systick = osKernelSysTick();

        osDelayUntil(&systick,2);
    }
}
