//
// Created by kun on 25-7-9.
//

#include "vofa.h"
#include <stdio.h>
#include <string.h>

#include "ctl_chassis.h"
#include "leg_angular_predictor.h"
#include "slip_detector.h"
#include "User_Task.h"


/* JustFloat协议发送遥控器通道值
 * @param huart: 串口句柄
 * @param rc_channels: 遥控器通道值数组（浮点数格式）
 * @param num_channels: 通道数量（建议不超过VOFA_CHANNELS）
 * @return HAL_StatusTypeDef: HAL_OK表示成功，其他表示失败
 */
extern INS_Info_Typedef INS_Info;
extern ist8310_real_data_t ist8310_Info;
extern dm8009_motor_measure_t motor_joint[4];
extern lk9025_motor_measure_t motor_right, motor_left;
Vofa_Frame_t vofa_tx;

HAL_StatusTypeDef Vofa_Send_Chassis(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,dm8009_motor_measure_t motor_joint[], chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

    .data = {
        // INS_Info.yaw_angle,
        // INS_Info.pit_angle,
        // INS_Info.rol_angle,
        //核心状态变量
        // chassis->chassis_yaw,
        chassis->chassis_pitch,
        chassis->chassis_roll,
        chassis->jump_state.jump_flag,
        chassis->jump_state.jump_stage,
        chassis->is_conversely,
        chassis->touchingGroung,

        //力控制变量 (分析失控原因)
        chassis->left_support_force,
        chassis->right_support_force,
        chassis->leg_tor,
        chassis->wheel_tor,

        //腿部状态变量 (分析动作细节)
        // chassis->right_leg.leg_angle,
        // chassis->right_leg.leg_length,
        // chassis->left_leg.leg_angle,
        // chassis->left_leg.leg_length,
        // chassis->state_ref.phi_dot
        // chassis->right_leg.front_joint.tor_set,
        // chassis->right_leg.back_joint.tor_set,
        // chassis->left_leg.front_joint.tor_set,
        // chassis->left_leg.back_joint.tor_set,
        //
        // motor_joint[0].pos,
        // motor_joint[1].pos,
        // motor_joint[2].pos,
        // motor_joint[3].pos,
    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_joint_angle(UART_HandleTypeDef *huart, const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

        .data = {


            //腿部状态变量 (分析动作细节)
            // chassis->right_leg.leg_angle,
            // chassis->right_leg.leg_length,
            // chassis->left_leg.leg_angle,
            // chassis->left_leg.leg_length,
            // chassis->state_ref.phi_dot
            // chassis->right_leg.front_joint.tor_set,
            // chassis->right_leg.back_joint.tor_set,
            // chassis->left_leg.front_joint.tor_set,
            // chassis->left_leg.back_joint.tor_set,
            //
            motor_joint[0].pos,
            motor_joint[1].pos,
            motor_joint[2].pos,
            motor_joint[3].pos,
            chassis->left_leg.wheel_motor.speed_set,
            chassis->right_leg.wheel_motor.speed_set,
           motor_left.speed,
            motor_right.speed,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
// HAL_StatusTypeDef Vofa_Send_Chassis_CMD(UART_HandleTypeDef *huart, const chassis_move_t* chassis)
// {
//
//
//     Vofa_Frame_t frame={
//
//         .data = {
//             gimbal_chassis_comm->gimbal_cmd.chassis_mode_cmd,
//             gimbal_chassis_comm->gimbal_cmd.target_speed_x,
//             gimbal_chassis_comm->gimbal_cmd.target_speed_w_z,
//             gimbal_chassis_comm->gimbal_cmd.target_length,
//             gimbal_chassis_comm->gimbal_cmd.spinning_cmd,
//             gimbal_chassis_comm->gimbal_cmd.gimbal_yaw_angle,
//             gimbal_chassis_comm->gimbal_cmd.jump_cmd,
//             gimbal_chassis_comm->gimbal_cmd.roll_angle,
//             gimbal_chassis_comm->gimbal_cmd.reserved1,
//             // gimbal_chassis_comm->gimbal_cmd.reserved2
//             // Chassis_cmd->ch[0] ,
//             // Chassis_cmd->ch[1] ,
//             // Chassis_cmd->ch[2] ,
//             // Chassis_cmd->ch[3] ,
//             // Chassis_cmd->s[0],
//             // Chassis_cmd->s[1],
//             // Chassis_cmd->s[2],
//             // Chassis_cmd->s[3],
//             // Chassis_cmd->x,
//             // Chassis_cmd->w_z,
//             // Chassis_cmd->roll_1,
//             // Chassis_cmd->roll_2,
//             // Chassis_cmd->left_switch_1,
//             // Chassis_cmd->right_switch_1,
//             // Chassis_cmd->left_switch_2,
//             // Chassis_cmd->right_switch_2,
//         }, // 1初始化数据数组
//             .tail = VOFA_TAIL // 设置JustFloat协议尾部
//         };
//     HAL_StatusTypeDef status;
//
//
//     // 发送整个帧（避免逐字节发送，提高效率）
//     status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
//     return status;
// }
HAL_StatusTypeDef Vofa_Send_System(UART_HandleTypeDef *huart, const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

        .data = {

            chassis->left_leg.wheel_motor.speed,
            chassis->right_leg.wheel_motor.speed,
            chassis->left_leg.wheel_motor.give_current,
            chassis->right_leg.wheel_motor.give_current,

        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_PC_Ctrl_Info(UART_HandleTypeDef *huart, const PC_Ctrl_Info_t *pc_ctrl_info)
{


    Vofa_Frame_t frame={

        .data = {
            // pc_ctrl_info->rc.ch[0],
            // pc_ctrl_info->rc.ch[1],
            // pc_ctrl_info->rc.ch[2],
            // pc_ctrl_info->rc.ch[3],
            // pc_ctrl_info->rc.mode_sw,
            // pc_ctrl_info->rc.pause,
            // pc_ctrl_info->rc.fn_1,
            // pc_ctrl_info->rc.fn_2,
            // pc_ctrl_info->rc.wheel,
            // pc_ctrl_info->rc.trigger,
            pc_ctrl_info->mouse.x,
            pc_ctrl_info->mouse.y,
            pc_ctrl_info->mouse.z,
            pc_ctrl_info->mouse.press_l,
            pc_ctrl_info->mouse.press_r,
            pc_ctrl_info->mouse.press_m,
            pc_ctrl_info->key.v,


        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_gimbal_Ctrl(UART_HandleTypeDef *huart, const gimbal_ctrl_frame_t* *ctrl_info)
{


    Vofa_Frame_t frame={

        .data = {
            // ctrl_info


        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Tor(UART_HandleTypeDef *huart, const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

        .data = {

            // chassis->jump_state.jump_comtorque[0],
            // chassis->jump_state.jump_comtorque[1],
            // chassis->jump_state.jump_comtorque[2],
            // chassis->jump_state.jump_comtorque[3],
            chassis->left_support_force,
            chassis->left_leg.leg_length,
            chassis->right_support_force,
            chassis->right_leg.leg_length,
            // chassis->left_leg.length_dot,
            // chassis->tor_vector[0],
            // chassis->tor_vector[1],
            // chassis->tor_vector[2],
            // chassis->tor_vector[3],
            chassis->left_leg.front_joint.tor_set,
            chassis->left_leg.back_joint.tor_set,
            // chassis->right_leg.back_joint.tor_set,
            chassis->left_leg_real_support,
            chassis->right_leg_real_support,
            // chassis->right_leg.front_joint.tor_set,
            chassis->jump_state.jump_flag,
            chassis->left_leg.length_dot,
            chassis->jump_state.jump_stage,
            chassis->touchingGroung,
            chassis->state_set.theta,
            chassis->state_ref.theta,
            // chassis->right_leg.wheel_motor.give_current,
            // chassis->left_leg.wheel_motor.give_current,

            // chassis->jump_state.F0,
            // chassis->jump_state.Tp,
            // chassis->jump_state.L0,
            // chassis->jump_state.Phi0,
            // chassis->jump_state.Fee[0],
            // chassis->jump_state.Fee[1],
            // chassis->left_leg.wheel_motor.speed,
            // chassis->right_leg.wheel_motor.speed,
            // chassis->left_leg.wheel_motor.give_current,
            // chassis->right_leg.wheel_motor.give_current,

        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_New_Chassis_Data(UART_HandleTypeDef *huart, const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

        .data = {
             // chassis->is_conversely,
        // chassis->left_leg.touching_ground,
        // chassis->right_leg.touching_ground,
       //  chassis->left_leg.front_joint.angle,
       // chassis->left_leg.back_joint.angle,
        // chassis->touchingGroung,
        //力控制变量 (分析失控原因)
        // chassis->l_force,
        // chassis->r_force,
        // chassis->left_support_force,  //腿支持力
        // chassis->right_support_force,
            // chassis->left_leg.leg_angle,
            // chassis->right_leg.leg_angle,
        // chassis->leg_tor,
        // chassis->wheel_tor,

        //腿部状态变量 (分析动作细节)
        // chassis->right_leg.leg_angle,
        // chassis->left_leg.leg_angle,
            // chassis->left_leg_real_support,
            // chassis->right_leg_real_support,
            // chassis->left_support_force,  //腿支持力
     // chassis->right_support_force,
        // chassis->left_leg.leg_length,
        // chassis->right_leg.leg_length, //腿长度
            // chassis->left_leg.touching_ground,
       // chassis->right_leg.touching_ground,
        // chassis->left_leg.length_dot,
        // chassis->right_leg.length_dot,
        // chassis->left_leg_real_support,
        // chassis->right_leg_real_support,
        // chassis->left_leg.leg_angle,
        // chassis->right_leg.leg_angle,
       //  chassis->right_leg.back_joint.angle,
       // chassis->right_leg.front_joint.angle,
        // chassis->state_ref.x_dot,
        // chassis->jump_state.jump_stage,

        // chassis->leg_length_in_sky,
        // chassis->chassis_imu_accel[]
        // chassis->left_leg.front_joint.tor_set,
        // chassis->left_leg.back_joint.tor_set,
        // chassis->right_leg.front_joint.tor_set,
        // chassis->right_leg.back_joint.tor_set,

            // chassis->touchingGroung,
        // chassis->jump_state.jump_stage,
        // chassis->jump_state.current_time,
        // chassis->jump_state.takeoff_start_time,
        // chassis->jump_state.takeoff_time,
        // chassis->jump_state.landing_time,
        // chassis->jump_state.last_jump_finish_time,
        // chassis->left_leg.leg_length_set ,
        // chassis->right_leg.leg_length_set,

        // chassis->state_set.phi,
        // chassis->state_ref.phi,
        // chassis->state_set.theta,
        // chassis->state_ref.theta,

        // chassis->state_ref.x_dot,
            // chassis->slip_detector->left.confidence,
        // chassis->slip_detector->right.confidence,
        // chassis->state_set.x,
        // chassis->state_set.x_dot,
        // chassis->state_set.theta,
        // chassis->state_set.phi,
        // chassis->jump_state.left_target_comp ,
        // chassis->jump_state.right_target_comp,
        // chassis->jump_state.jump_height[0],
        // chassis->jump_state.jump_height[1],
       //      chassis->left_support_force,  //腿支持力
       // chassis->right_support_force,
       chassis->left_leg.front_joint.tor_set,
       chassis->left_leg.back_joint.tor_set,
       // chassis->right_leg.front_joint.tor_set,
       // chassis->right_leg.back_joint.tor_set,
            // chassis->chassis_imu_accel[2],
            // chassis->state_set.theta,
            // chassis->state_set.x,
            chassis->state_set.x_dot,
            chassis->state_set.phi,
            chassis->left_leg.leg_length_set,
            chassis->right_leg.leg_length_set,
            chassis->state_ref.theta,
            chassis->state_ref.theta_dot,
            chassis->state_ref.x,
            chassis->state_ref.x_dot,
            chassis->state_ref.phi,
            chassis->state_ref.phi_dot,
            chassis->left_leg.leg_length,
            chassis->right_leg.leg_length, //腿长度
            // chassis->left_leg_real_support,
       // chassis->right_leg_real_support,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}

HAL_StatusTypeDef Vofa_Send_Data(UART_HandleTypeDef *huart,const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

    .data = {

        //核心状态变量
        // chassis->chassis_yaw,
        chassis->chassis_pitch,
        chassis->chassis_roll,
        chassis->chassis_imu_accel[2],
        chassis->is_conversely,
        // chassis->left_leg.touching_ground,
        // chassis->right_leg.touching_ground,
       //  chassis->left_leg.front_joint.angle,
       // chassis->left_leg.back_joint.angle,
        chassis->touchingGroung,
        //力控制变量 (分析失控原因)
        chassis->l_force,
        chassis->r_force,
        chassis->left_support_force,  //腿支持力
        chassis->right_support_force,
        chassis->leg_tor,
        chassis->wheel_tor,

        //腿部状态变量 (分析动作细节)
        // chassis->right_leg.leg_angle,
        // chassis->left_leg.leg_angle,
        chassis->left_leg.leg_length,
        chassis->right_leg.leg_length, //腿长度
        chassis->left_leg.length_dot,
        chassis->right_leg.length_dot,
        chassis->left_leg_real_support,
        chassis->right_leg_real_support,
        // chassis->left_leg.leg_angle,
        // chassis->right_leg.leg_angle,
       //  chassis->right_leg.back_joint.angle,
       // chassis->right_leg.front_joint.angle,
        chassis->state_ref.x_dot,
        chassis->jump_state.jump_stage,

        chassis->leg_length_in_sky,
        // chassis->chassis_imu_accel[]
        // chassis->left_leg.front_joint.tor_set,
        // chassis->left_leg.back_joint.tor_set,
        // chassis->right_leg.front_joint.tor_set,
        // chassis->right_leg.back_joint.tor_set,

        chassis->jump_state.jump_flag,
        chassis->jump_state.current_time,
        chassis->jump_state.takeoff_start_time,
        chassis->jump_state.takeoff_time,
        chassis->jump_state.landing_time,
        chassis->jump_state.last_jump_finish_time,
        // chassis->left_leg.leg_length_set ,
        // chassis->right_leg.leg_length_set,

        // chassis->state_set.phi,
        chassis->state_ref.phi,
        // chassis->state_set.theta,
        chassis->state_ref.theta,
        // chassis->slip_detector->left.confidence,
        // chassis->slip_detector->right.confidence,
        chassis->state_ref.x_dot,
        // chassis->state_set.x,
        // chassis->state_set.x_dot,
        // chassis->state_set.theta,
        // chassis->state_set.phi,
        // chassis->jump_state.left_target_comp ,
        // chassis->jump_state.right_target_comp,
        chassis->jump_state.jump_height[0],
        chassis->jump_state.jump_height[1],
        chassis->left_leg.front_joint.tor_set,
       chassis->left_leg.back_joint.tor_set,
       chassis->right_leg.front_joint.tor_set,
       chassis->right_leg.back_joint.tor_set,
        // chassis->ground_force,
        // chassis->left_leg.leg_length_set,
        // chassis->right_leg.leg_length_set,
        // chassis->state_ref.phi_dot
        // chassis->right_leg.front_joint.tor_set,
        // chassis->right_leg.back_joint.tor_set,
        // chassis->left_leg.front_joint.tor_set,
        // chassis->left_leg.back_joint.tor_set,
        //
        // chassis->left_leg.front_joint.angle,
        // chassis->left_leg.back_joint.angle,
        // chassis->right_leg.front_joint.angle,
        // chassis->right_leg.back_joint.angle,
        // motor_joint[0].pos,
        // motor_joint[1].pos,
        // motor_joint[2].pos,
        // motor_joint[3].pos,
    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Calibrate(UART_HandleTypeDef *huart,const chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

    .data = {



        // chassis->left_leg.leg_length_set,
        // chassis->right_leg.leg_length_set,
        // chassis->state_ref.phi_dot
        // chassis->right_leg.front_joint.tor_set,
        // chassis->right_leg.back_joint.tor_set,
        // chassis->left_leg.front_joint.tor_set,
        // chassis->left_leg.back_joint.tor_set,
        //
        chassis->left_leg.front_joint.angle,
        chassis->left_leg.back_joint.angle,
        chassis->right_leg.back_joint.angle,
        chassis->right_leg.front_joint.angle,
        motor_joint[0].pos,
        motor_joint[1].pos,
        motor_joint[2].pos,
        motor_joint[3].pos,
    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Balance(UART_HandleTypeDef *huart, chassis_move_t* chassis)
{


    Vofa_Frame_t frame={

    .data = {

        //核心状态变量
        chassis->chassis_pitch,
        chassis->chassis_roll,
        chassis->is_conversely,
        chassis->left_leg.touching_ground,
        chassis->right_leg.touching_ground,
        //力控制变量 (分析失控原因)



        //腿部状态变量 (分析动作细节)
        chassis->left_leg.leg_length,
        chassis->right_leg.leg_length, //腿长度

        chassis->l_force,
        chassis->r_force,
        chassis->left_support_force,  //腿支持力
       chassis->right_support_force,
        chassis->tor_vector[0],
        chassis->tor_vector[1],
        chassis->tor_vector[2],
        chassis->tor_vector[3],
        chassis->left_leg.front_joint.tor_set,
        chassis->left_leg.back_joint.tor_set,
        chassis->right_leg.front_joint.tor_set,
        chassis->right_leg.back_joint.tor_set,
        chassis->leg_tor,
       chassis->wheel_tor,

    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Slip(UART_HandleTypeDef *huart, chassis_move_t* chassis,SlipDetector_t *detector)
{
    // SlipFlag slip_flag = get_slip_flag();
    // float l_confidance= get_confidence_left();
    // float r_confidance= get_confidence_right();
    // float R_scale_v= get_r_scale_velocity();
    // float R_scale_a=get_r_scale_accel();
    Vofa_Frame_t frame={

    .data = {

        //核心状态变量
        chassis->chassis_pitch,
        chassis->chassis_roll,
        chassis->chassis_imu_accel[0],
        chassis->is_conversely,
        chassis->touchingGroung,
        //力控制变量 (分析失控原因)
        chassis->l_force,
        chassis->r_force,
        chassis->left_support_force,  //腿支持力
        chassis->right_support_force,

        chassis->leg_tor,
        chassis->wheel_tor,

        //腿部状态变量 (分析动作细节)
        chassis->left_leg.leg_length,
        chassis->right_leg.leg_length, //腿长度

        chassis->state_ref.x_dot,
        detector->left.confidence,
        detector->right.confidence,
        detector->slip_flag,
        detector->r_scale_velocity,
        detector->r_scale_accel,
        detector->imu_accel_x,
        detector->left_torque_cmd,
        detector->right_torque_cmd,
        // l_confidance,
        // r_confidance,
        // slip_flag,
        // R_scale_v,
        // R_scale_a

    }, // 1初始化数据数组
        .tail = VOFA_TAIL // 设置JustFloat协议尾部
    };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_INS(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info,ist8310_real_data_t ist8310_Info)
{


    Vofa_Frame_t frame={

        .data = {
            INS_Info.yaw_angle,
            INS_Info.pit_angle,
            INS_Info.rol_angle,
            // ist8310_Info.raw_mag[0],
            // ist8310_Info.raw_mag[1],
            // ist8310_Info.raw_mag[2],
            ist8310_Info.calibrated_mag[0],
            ist8310_Info.calibrated_mag[1],
            ist8310_Info.calibrated_mag[2],
            // ist8310_Info.mag_max[0],
            // ist8310_Info.mag_max[1],
            // ist8310_Info.mag_max[2],
            // ist8310_Info.mag_min[0],
            // ist8310_Info.mag_min[1],
            // ist8310_Info.mag_min[2],
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Pred(UART_HandleTypeDef *huart,chassis_move_t* chassis )
{


    Vofa_Frame_t frame={

        .data = {
            // chassis->leg_dynamics_predictor.theta_pred[0],
            // chassis->leg_dynamics_predictor.theta_pred[1],
            // chassis->leg_dynamics_predictor.theta_dot_pred[0],
            // chassis->leg_dynamics_predictor.theta_dot_pred[1],
            // chassis->leg_dynamics_predictor.theta_ddot_pred[0],
            // chassis->leg_dynamics_predictor.theta_ddot_pred[1],
            chassis->state_ref.theta,
            chassis->state_ref.theta_dot,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Theata(UART_HandleTypeDef *huart,chassis_move_t* chassis )
{


    Vofa_Frame_t frame={

        .data = {
            chassis->left_leg.leg_length,
            chassis->right_leg.leg_length,
            chassis->leg_length,
            chassis->state_ref.theta,
            chassis->state_ref.theta_dot,
            chassis->state_ref.theta_ddot,
            chassis->left_leg.angle_dot,
            chassis->right_leg.angle_dot,
            chassis->leg_tor,
            chassis->left_leg.leg_angle,
            chassis->right_leg.leg_angle,


        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Theata_pre(UART_HandleTypeDef *huart,const chassis_move_t* chassis ,const LegPredictor_t *predictor,const SlipDetector_t *detector)
{


    Vofa_Frame_t frame={

        .data = {
            chassis->left_leg.leg_length,
            chassis->right_leg.leg_length,
            chassis->leg_length,
            chassis->state_ref.theta,
            chassis->state_ref.theta_dot,
            chassis->state_ref.theta_ddot,
            chassis->left_leg.angle_dot,
            chassis->right_leg.angle_dot,
            chassis->leg_tor,
            chassis->left_leg.leg_angle,
            chassis->right_leg.leg_angle,
            predictor->K_adjust,
            predictor->theta_pred_left,
            predictor->omega_pred_left,
            predictor->theta_pred_right,
            predictor->omega_pred_right,
            predictor->comp_left,
            predictor->comp_right,
            detector->slip_flag,
            detector->left.confidence,
            detector->right.confidence,
            chassis->chassis_pitch,
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}
HAL_StatusTypeDef Vofa_Send_Q(UART_HandleTypeDef *huart, INS_Info_Typedef INS_Info, Quaternion_Info_Typedef *Quaternion_Info)
{


    Vofa_Frame_t frame={

        .data = {
            // Quaternion_Info->quat[0],
            // Quaternion_Info->quat[1],
            // Quaternion_Info->quat[2],
            // Quaternion_Info->quat[3],
            Quaternion_Info->EulerAngle[0],
            Quaternion_Info->EulerAngle[1],
            Quaternion_Info->EulerAngle[2],
            INS_Info.yaw_angle,
            INS_Info.pit_angle,
            INS_Info.rol_angle,
            // INS_Info.yaw_gyro,
            // Quaternion_Info->yaw_mag*57.295779513f,
            // Quaternion_Info->dyaw,
            // Quaternion_Info->deviate[2],
            // ist8310_Info.raw_mag[0],
            // ist8310_Info.raw_mag[1],
            // ist8310_Info.raw_mag[2],
            // ist8310_Info.calibrated_mag[0],
            // ist8310_Info.calibrated_mag[1],
            // ist8310_Info.calibrated_mag[2],
            // ist8310_Info.mag_max[0],
            // ist8310_Info.mag_max[1],
            // ist8310_Info.mag_max[2],
            // ist8310_Info.mag_min[0],
            // ist8310_Info.mag_min[1],
            // ist8310_Info.mag_min[2],
        }, // 1初始化数据数组
            .tail = VOFA_TAIL // 设置JustFloat协议尾部
        };
    HAL_StatusTypeDef status;


    // 发送整个帧（避免逐字节发送，提高效率）
    status = HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Vofa_Frame_t), 100);
    return status;
}

void uart_printf(UART_HandleTypeDef *huart, const char *fmt, ...)
{
    char buffer[128]; // 根据需要可增大
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

