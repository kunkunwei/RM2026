//
// Created by kun on 25-7-27.
//

#include "stepper_motor.h"

#include <stddef.h>

Stepper_motor_measure_t pitch_motor,yaw_motor; // 两个步进电机



//返回指针
// StepperMotor_Info_t* get_stepper_motor_info(uint8_t index) {
//     if (index < 2) {
//         return &StepperMotor[index];
//     }
//     return NULL; // 超出范围返回NULL
// }
Stepper_motor_measure_t* get_yaw_motor_info() {
    return &yaw_motor;
}
Stepper_motor_measure_t* get_pitch_motor_info() {
    return &pitch_motor;
}