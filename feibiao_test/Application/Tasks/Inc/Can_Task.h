#ifndef __CAN_TASK__
#define __CAN_TASK__

#include "main.h"
#include "old_pid.h"
// 3508电机速度环PID
#define FEIBIAO_M3505_PID_KP       3000.0f
#define FEIBIAO_M3505_PID_KI       0.1f
#define FEIBIAO_M3505_PID_KD       1.0f
#define FEIBIAO_M3505_PID_MAX_OUT  12000.0f
#define FEIBIAO_M3505_PID_MAX_IOUT 800.0f

// YAW电机速度环PID
#define YAW_6020_PID_KP       12000.0f
#define YAW_6020_PID_KI       4.0f
#define YAW_6020_PID_KD       0.0f
#define YAW_6020_PID_MAX_OUT  12000.0f
#define YAW_6020_PID_MAX_IOUT 1200.0f

// ROLL电机速度环PID
#define ROLL_6020_PID_KP       10000.0f
#define ROLL_6020_PID_KI       7.0f
#define ROLL_6020_PID_KD       2.0f
#define ROLL_6020_PID_MAX_OUT  12000.0f
#define ROLL_6020_PID_MAX_IOUT 1200.0f

#define TARGET_SPD             6.0f
#define RC_TO_ADDYAW           0.000025f
typedef struct
{
    dji_motor_measure_t *roll_6020;
    // TickType_t start_load_feibiao_time;
    bool_t is_load_feibiao;
    float roll_6020_load_pos;
    float target_pos;
    PidTypeDef roll_6020_pid;

    dji_motor_measure_t *yaw_6020;
    float next_yaw_pos;
    float tmp_yaw;
    bool_t cross_flag;
    PidTypeDef yaw_6020_pid;

    dji_motor_measure_t *pull_3508;
    float target_w_3508;
    bool_t is_pull;
    PidTypeDef pull_3508_pid;

    const Remote_Info_Typedef *feibiao_rc;
} Feibiao_control_t;


void Can_Task(void const * argument);

#endif // !__CAN_TASK__
