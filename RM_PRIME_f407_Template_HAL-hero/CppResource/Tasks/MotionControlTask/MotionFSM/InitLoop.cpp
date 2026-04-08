#include "MotionFSM.h"
#include "MotionControl.h"

using namespace MotionFSM;

static Referee &hreferee = Referee::getInstance();

StateLoopArg MotionFSM::InitLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg& cur_sta){
    InitFlag.InitNI = false;

    bool gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
    bool chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;

    // 当底盘和云台都上电时，设置InitFlag.InitNI为true，让RemoteControlMidwareLoop处理状态转换
    if(chassis_on && gimbal_on)
    {
        InitFlag.InitNI = true;
    }

    return {
            {{0,0,0}},
            {{cur_sta.YawSta.pos,0}},
            {{cur_sta.PithSta.pos,0}},
            cur_sta.taget_pitch
    };
}
