#include "MotionControl.h"
#include "arm_math.h"

using namespace MotionParameter;

#define InInterval(val, center, R) ( (val) < ((center)+(R)) ) && ( (val) > ((center)-(R)) )

namespace GimbalControl{

    DM4310 YawMotor(&hcan1, 2, 4*PI, 30.f,10.f);
    DM4310 PithMotor(&hcan2, 1);

    float YawZero = 0.0f;
    // float YawZero = 3.5f;
    float PitchZero = 0.5f;
    static bool isPowerOn = false;

    //将角度规范化到 -PI 到 PI
    float angleMod(float angle){
        while (angle > PI) angle -= 2.f * PI;
        while (angle < -PI) angle += 2.f * PI;
        return angle;
    }


    AxisState getYawState(){

        if(!isPowerOn){
            return {{0,0}};
        }

        return {
                {YawMotor.getMotorState().pos * YawGearRate,
                 YawMotor.getMotorState().vel * YawGearRate}
        };
    }

    AxisState getPithState(){

        if(!isPowerOn){
            return {{0,0}};
        }

        return {
            {(PithMotor.getMotorState().pos - PitchZero)* PithGearRate,
            PithMotor.getMotorState().vel * PithGearRate}
        };
    }

    void NotifyPowerSate(bool s){
        isPowerOn = s;
    }

    void setYawRelative(AxisState target_s){

        // 如果电源关闭，不发送指令给电机
        if(!isPowerOn){
            YawMotor.setMITcmd(0,0,0,0,0);
            return;
        }

        //角速度限制
        float current_omega = getYawState().omega;
        float delta_omega = target_s.omega - current_omega;
        float max_delta_omega = GIMBAL_MAX_ALPHA*T_SAMPLE;
        if(delta_omega > max_delta_omega) delta_omega = max_delta_omega;
        if(delta_omega < -max_delta_omega) delta_omega = -max_delta_omega;
        target_s.omega = current_omega + delta_omega;

        //位置限制与插值
        float  current_pos = angleMod(getYawState().pos);
        float des_pos = angleMod(target_s.pos);

        float dpos = GimbalControl::angleMod(  des_pos - current_pos);
        int dir = 1;
        float alpha = 0;

        //符号判断
        if(dpos < -0.f) dir = -1;
        //线性插值（可改成其他插值函数）
        if(dpos * (float)dir > GIMBAL_RAD_VEL){
            alpha = 1;
        }
        else if (dpos * (float)dir < GIMBAL_RAD_POS){
            alpha = 0;
        }
        else {
            alpha = (dpos * (float)dir - GIMBAL_RAD_POS) / (GIMBAL_RAD_VEL - GIMBAL_RAD_POS);
        }

        target_s.pos = current_pos * alpha + des_pos * (1.f - alpha);
        float kp_omega = alpha * (GIMBAL_MOVE_SPEED * (float)dir);

        float angle_err = angleMod(target_s.pos - current_pos);
        //限制幅度
        float except_t = angle_err * GIMBAL_YAW_KP;
        if(except_t > 9.9f)except_t = 9.9f;
        if(except_t < -9.9f)except_t = -9.9f;

        if(target_s.omega > 20.f) target_s.omega = 20.f;
        if(target_s.omega < -20.f) target_s.omega = -20.f;

        YawMotor.setMITcmd(0,(target_s.omega+kp_omega) * (1.f/YawGearRate), 0, GIMBAL_YAW_KD, except_t);
//        YawMotor.setMITcmd(0,0,0,0,0);
    }

    void setPithRelative(AxisState target_s){

        // 如果电源关闭，不发送指令给电机
        if(!isPowerOn){
            PithMotor.setMITcmd(0,0,0,0,0);
            return;
        }

        if(target_s.pos > GIMBAL_PITCH_UPPER) target_s.pos = GIMBAL_PITCH_UPPER;
        if(target_s.pos < GIMBAL_PITCH_LOWER) target_s.pos = GIMBAL_PITCH_LOWER;

        float pitch_rad = getPithState().pos;
        float error_pos = target_s.pos - pitch_rad;
        float epo = error_pos*GIMBAL_PITCH_KI;
        if(epo > 5.f) epo = 5.f;
        if(epo < -5.f) epo = -5.f;
        float error_omega = target_s.omega - getPithState().omega + epo;

        //重力补偿
        float c,s;
        arm_sin_cos_f32(pitch_rad, &s, &c);
        float t_compen = c*GIMBAL_PITCH_CMX+s*GIMBAL_PITCH_CMY;
        float tor = error_pos * GIMBAL_PITCH_KP +
                error_omega * GIMBAL_PITCH_KD +
                t_compen;

        //tor = t_compen;

        if(tor > 9.9f) tor = 9.9f;
        if(tor < -9.9f) tor = -9.9f;

        PithMotor.setMITcmd(0,0,0,0,tor);
    }

}
