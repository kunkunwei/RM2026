#include "MotionFSM.h"
#include "arm_math.h"
#include "NanoMsg.h"

using namespace MotionFSM;
using namespace MotionParameter;

static auto &haimnano = NanoMsg::getInstance();
static float Abs_deg_recode = 0;
StateLoopArg MotionFSM::AutoAimLoop(const volatile RCcmd_t* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){
    if(InitFlag.AutoAimNI){
        Abs_deg_recode = hINS.getAngle().yaw;
        InitFlag.AutoAimNI = false;
    }

    float relativeYaw = GimbalControl::angleMod(cur_sta.YawSta.pos-GimbalControl::YawZero) * (180.f/PI);
    float cos_yaw, sin_yaw;
    arm_sin_cos_f32(relativeYaw, &sin_yaw, &cos_yaw);

    ChassisControl::MoveState AbsMove = {
            {(float)RCsta->ch[3] * CHASSIS_K_CHX,
             -(float)RCsta->ch[2] * CHASSIS_K_CHY,
             -(float)RCsta->ch[0] * CHASSIS_K_OMEGA}
    };

    float chassis_omega_ref = 0;
    float dpos = GimbalControl::angleMod(cur_sta.YawSta.pos - GimbalControl::YawZero);
    int dir = 1;
    if(dpos < 0) dir = -1;
    if(dpos * (float)dir > CHASSIS_FOLLOW_DES){
        chassis_omega_ref = CHASSIS_FOLLOW_OMEGA * (float)dir;
    }

    ChassisControl::MoveState target_state = {
            {AbsMove.vx * cos_yaw - AbsMove.vy * sin_yaw,
             AbsMove.vx * sin_yaw + AbsMove.vy * cos_yaw,
             chassis_omega_ref}
    };

    auto angcmd = haimnano.getControlCmd();
    float real_omega = AbsMove.omega * GIMBAL_K_LEAD_OMEGA;

    auto PitchData = cur_sta.PithSta;
    float ref_ang =  PitchData.pos + angcmd.pitch - hINS.getAngle().pitch;
    Abs_deg_recode = angcmd.yaw;
    //Abs_deg_recode = GimbalControl::angleMod(Abs_deg_recode);

    auto YawData = cur_sta.YawSta;
    float taget_pos = YawData.pos + Abs_deg_recode - hINS.getAngle().yaw;

    StateLoopArg rel = {
            target_state,
            {{taget_pos, -cur_sta.ChassisSta.omega * GIMBAL_K_OMEGA_FORWARD}},
            {{ref_ang, 0}},
            ref_ang
    };

    return rel;
}
