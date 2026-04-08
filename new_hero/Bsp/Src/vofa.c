//
// Created by kun on 25-7-9.
//

#include "../Inc/vofa.h"
#include <stdio.h>
#include <string.h>




/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
/* INS_Info 已迁移为 static，通过 get_ins_info_point() 访问，此处无需 extern 声明 */

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 根据需要可增大
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}
HAL_StatusTypeDef Vofa_Send_PC_Ctrl_Info(UART_HandleTypeDef *huart, const PC_Ctrl_Info_t *pc_ctrl_info)
{


    Vofa_Frame_t frame={

        .data = {
            pc_ctrl_info->rc.ch[0],
            pc_ctrl_info->rc.ch[1],
            pc_ctrl_info->rc.ch[2],
            pc_ctrl_info->rc.ch[3],
            pc_ctrl_info->rc.mode_sw,
            pc_ctrl_info->rc.pause,
            pc_ctrl_info->rc.fn_1,
            pc_ctrl_info->rc.fn_2,
            pc_ctrl_info->rc.wheel,
            pc_ctrl_info->rc.trigger,
            // pc_ctrl_info->mouse.x,
            // pc_ctrl_info->mouse.y,
            // pc_ctrl_info->mouse.z,
            // pc_ctrl_info->mouse.press_l,
            // pc_ctrl_info->mouse.press_r,
            // pc_ctrl_info->mouse.press_m,
            // pc_ctrl_info->key.v,
            // pc_ctrl_info->key.set.W,
            // pc_ctrl_info->key.set.A,
            // pc_ctrl_info->key.set.S,
            // pc_ctrl_info->key.set.D,


        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_shoot_Info(UART_HandleTypeDef *huart, const gimbal_t *gimbal)
{


    Vofa_Frame_t frame={

        .data = {
            // gimbal->gimbal_PC_RC->rc.wheel,
            gimbal->gimbal_shoot.shoot_mode,
            // gimbal->gimbal_shoot.Fric_rad[0],
            // gimbal->gimbal_shoot.Fric_rad[1],
            // gimbal->gimbal_shoot.Fric_rad[2],
            // gimbal->gimbal_shoot.Fric_rad[3],
            // gimbal->gimbal_shoot.Fric_motor[0]->target_current,
            // gimbal->gimbal_shoot.Fric_motor[1]->target_current,
            // gimbal->gimbal_shoot.Fric_motor[2]->target_current,
            // gimbal->gimbal_shoot.Fric_motor[3]->target_current,
            // gimbal->gimbal_shoot.shoot_right_rad,
            // gimbal->gimbal_shoot.shoot_left_rad,
            // gimbal->gimbal_shoot.reverse_count,
            gimbal->gimbal_shoot.pull_target_speed,
            gimbal->gimbal_shoot.pull_motor->target_current,
            gimbal->gimbal_shoot.pull_motor->ecd,
             gimbal->gimbal_shoot.pull_abs_pos,
            gimbal->gimbal_shoot.Pull_PID.out,
            gimbal->gimbal_shoot.block_count,
            gimbal->gimbal_shoot.bullet_done,
            gimbal->gimbal_shoot.set_angle,
            // gimbal->gimbal_pos.yaw_motor_measure->pos,
            // gimbal->gimbal_pos.pitch_motor_measure->pos,
            // gimbal->gimbal_pos.yaw_relattive_pos,
            // gimbal->gimbal_pos.pitch_relatiive_pos,
            // gimbal->gimbal_pos.yaw_target_pos,
            // gimbal->gimbal_pos.yaw_absolute_pos,
            // gimbal->gimbal_pos.usb_autoAim_ptr->minipc_target_yaw,


        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_chassis_Info(UART_HandleTypeDef *huart, const chassis_move_t *chassis)
{


    Vofa_Frame_t frame={

        .data = {
            chassis->mode.chassis_mode,
            // chassis->chassis_motor[0].chassis_motor_measure->current,
            // chassis->chassis_motor[1].chassis_motor_measure->current,
            // chassis->chassis_motor[2].chassis_motor_measure->current,
            // chassis->chassis_motor[3].chassis_motor_measure->current,
            // chassis->chassis_motor[3].chassis_motor_measure->target_current,
            chassis->state_set.vx,
            chassis->state_set.vy,
            chassis->state_set.wz,
            chassis->state_ref.gimbal_yaw_relative,
            chassis->state_ref.chassis_yaw_relative,
            // chassis->state_ref.vx,
            // chassis->state_ref.vy,
            // chassis->state_ref.wz,
            chassis->chassis_motor[0].speed_set,
            chassis->chassis_motor[0].target_current
            // chassis->chassis_pid.motor_speed_pid[3].out



        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}