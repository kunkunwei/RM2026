#include "MotionFSM.h"
#include "arm_math.h"

using namespace MotionFSM;

StateLoopArg MotionFSM::IdleLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg& cur_sta){
    if(InitFlag.IdleNI){
        InitFlag.IdleNI = false;
    }


    return {
            {{0,0,0}},
            {{cur_sta.YawSta.pos,0}},
            {{cur_sta.PithSta.pos,0}},
            cur_sta.taget_pitch
    };
}
