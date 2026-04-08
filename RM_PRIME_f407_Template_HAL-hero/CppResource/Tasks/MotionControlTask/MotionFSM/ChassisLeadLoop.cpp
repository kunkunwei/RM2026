#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;
using namespace MotionParameter;

StateLoopArg MotionFSM::ChassisLeadLoop(const volatile RCcmd_t* RCsta, INS_Device& hINS, const StateLoopArg& cur_sta){

    if(InitFlag.ChassisLeadNI){
        InitFlag.ChassisLeadNI = false;
    }

    ChassisControl::MoveState target_state = {
            {(float)RCsta->ch[3] * CHASSIS_K_CHX,
             -(float)RCsta->ch[2] * CHASSIS_K_CHY,
             -(float)RCsta->ch[0] * CHASSIS_K_OMEGA}
    };

    float pitch_omega = (float) RCsta->ch[1] * GIMBAL_K_CH_PITH;
    float taget_pitch = GimbalControl::angleMod(cur_sta.taget_pitch + pitch_omega*T_SAMPLE);

    StateLoopArg rel = {
            target_state,
            {{GimbalControl::YawZero, 0}},
            {{taget_pitch, pitch_omega}},
            taget_pitch
    };

    return rel;

}
