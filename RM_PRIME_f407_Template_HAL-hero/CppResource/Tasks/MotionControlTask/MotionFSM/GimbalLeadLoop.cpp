#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;
using namespace MotionParameter;

static float Abs_deg_recode = 0;
StateLoopArg MotionFSM::GimbalLeadLoop(const volatile RCcmd_t* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(InitFlag.GimbalLeadNI){
        Abs_deg_recode = hINS.getAngle().yaw;
        InitFlag.GimbalLeadNI = false;
    }

    float relativeYaw = GimbalControl::angleMod(cur_sta.YawSta.pos-GimbalControl::YawZero) * (180.f/PI);
    float cos_yaw, sin_yaw;
    arm_sin_cos_f32(relativeYaw, &sin_yaw, &cos_yaw);

    ChassisControl::MoveState AbsMove = {
            {(float)RCsta->ch[3] * CHASSIS_K_CHX,
             -(float)RCsta->ch[2] * CHASSIS_K_CHY,
             -(float)RCsta->ch[0] * CHASSIS_K_OMEGA}
    };
    float pitch_omega = (float) RCsta->ch[1] * GIMBAL_K_CH_PITH;
    float taget_pitch = GimbalControl::angleMod(cur_sta.taget_pitch + pitch_omega*T_SAMPLE);

    float chassis_omega_ref = 0;
    float dpos = GimbalControl::angleMod(cur_sta.YawSta.pos - GimbalControl::YawZero);
    int dir = 1;
    if(dpos < 0) dir = -1;
    if(dpos * (float)dir > CHASSIS_FOLLOW_DES){
        chassis_omega_ref = CHASSIS_FOLLOW_OMEGA * (float)dir;
    }
    //chassis_omega_ref = 0;
    ChassisControl::MoveState target_state = {
            {AbsMove.vx * cos_yaw - AbsMove.vy * sin_yaw,
             AbsMove.vx * sin_yaw + AbsMove.vy * cos_yaw,
             chassis_omega_ref}
    };

    float real_omega = AbsMove.omega * GIMBAL_K_LEAD_OMEGA;
    Abs_deg_recode += real_omega*T_SAMPLE;

    auto YawData = GimbalControl::getYawState();
    float taget_pos = GimbalControl::angleMod(YawData.pos + Abs_deg_recode - hINS.getAngle().yaw);

    StateLoopArg rel = {
            target_state,
            {{taget_pos, -cur_sta.ChassisSta.omega * GIMBAL_K_OMEGA_FORWARD}},
            {{taget_pitch, pitch_omega}},
            taget_pitch
    };


    return rel;
}