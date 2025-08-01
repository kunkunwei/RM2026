//
// Created by kun on 25-7-13.
//

#include "../Inc/user_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "stepper_can.h"
#include "usart.h"
#include "math.h"

extern osThreadId StartUserTaskHandle;
extern float target_yaw_angle, target_pitch_angle;
extern int32_t g_position_yaw, g_position_pitch;

#define POSITION_READ_PERIOD_MS 50   // 位置读取周期50ms（20Hz）

typedef struct {
    StepperMotor_Info_t* motor;
    StepperMotorID_e motor_id;
    TickType_t last_read_tick;
    bool position_reading;
    const char* name;
} MotorReader_t;

void User_Task(void const * argument)
{
    // 等待系统初始化完成
    osDelay(3000);

    // printf("User_Task Started - Position Reading Only\r\n");

    // 初始化电机读取器结构
    MotorReader_t readers[2] = {
        {&yaw_motor_Info, CAN1_YAW_MOTOR_ID, 0, false, "YAW"},
        {&pitch_motor_Info, CAN1_PITCH_MOTOR_ID, 0, false, "PITCH"}
    };

    uint8_t active_motor = 0; // 当前处理的电机索引
    TickType_t last_status_print = 0;
    uint32_t read_counter = 0;

    for(;;)
    {
        // 使用超时等待，确保任务能够持续运行
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25)); // 25ms超时

        TickType_t now = osKernelSysTick();
        MotorReader_t* reader = &readers[active_motor];

        // 位置读取逻辑 - 交替读取两个电机
        bool should_read_position = !reader->position_reading &&
                                   ((now - reader->last_read_tick) >= pdMS_TO_TICKS(POSITION_READ_PERIOD_MS));

        if (should_read_position) {
            // 清除可能卡住的标志位
            if (reader->position_reading) {
                reader->position_reading = false;
            }

            // printf("Reading %s Motor Position (ID:0x%03X)...\r\n", reader->name, reader->motor_id);

            if (StepperCAN_ReadPosition(reader->motor_id)) {
                reader->last_read_tick = now;
                reader->position_reading = true;
                read_counter++;
                // printf("%s Position Read Command Sent (Count: %lu)\r\n", reader->name, read_counter);
            } else {
                // 读取失败，缩短等待时间重试
                reader->position_reading = false;
                reader->last_read_tick = now - pdMS_TO_TICKS(POSITION_READ_PERIOD_MS / 2);
                // printf("%s Position Read Failed\r\n", reader->name);
            }
        }

        // 重置位置读取标志位（超时保护）
        if (reader->position_reading) {
            if ((now - reader->last_read_tick) > pdMS_TO_TICKS(200)) { // 200ms超时
                reader->position_reading = false;
                // printf("%s Position Read Timeout Reset\r\n", reader->name);
            }
        }



        // 交替处理两个电机以分散负载
        active_motor = (active_motor + 1) % 2;

        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

        // 给其他任务时间执行
        osDelay(10);
    }
}
