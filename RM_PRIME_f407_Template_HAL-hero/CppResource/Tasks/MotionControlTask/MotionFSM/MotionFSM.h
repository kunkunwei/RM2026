#ifndef F407_RM_TMPLATE_HAL_MOTIONFSM_H
#define F407_RM_TMPLATE_HAL_MOTIONFSM_H

#include "MotionControl.h"

using RCcmd_t = RemoteContrlMidware::VirtualRC_CMD;

namespace MotionFSM{

    struct StateLoopArg{
        ChassisControl::MoveState ChassisSta;
        GimbalControl::AxisState YawSta;
        GimbalControl::AxisState PithSta;
        float taget_pitch;
    };

    using StateHandler = StateLoopArg(*)(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    // 各 Loop 函数需统一 StateHandler 签名
    StateLoopArg InitLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    StateLoopArg IdleLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    StateLoopArg ChassisLeadLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    StateLoopArg GimbalLeadLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoAimLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);
    StateLoopArg AutoRotateLoop(const volatile RCcmd_t*, INS_Device&, const StateLoopArg&);

    inline StateHandler CurrentHandler = InitLoop;
    constexpr StateLoopArg DefaultStateArg = {
            {{0,0,0}},
            {{0,0}},
            {{0,0}},
            0.f
    };

    struct Flag{
        bool InitNI,IdleNI,ChassisLeadNI,GimbalLeadNI,AutoAimNI,AutoRotateNI;
    };

    volatile inline Flag InitFlag = {
            true,
            true,
            true,
            true,
            true,
            true
    };

    //状态定义
    struct Init{};
    struct Idle{};
    struct ChassisLead{};
    struct GimbalLead{};
    struct AutoAim{};
    struct AutoRotate{};

    //事件定义
    struct InitComplete{};
    struct IntoIdle{};
    struct IntoChassisLead{};
    struct IntoGimbalLead{};
    struct IntoAutoAim{};
    struct IntoAutoRotate{};

    struct PausePress{};
    struct LeftPress{};
    struct RightPress{};

    struct PowerOff{};

    struct motionTransition{
        auto operator()() const{
            using namespace boost::sml;
            return make_transition_table(
                    *state<Init> + event<InitComplete> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>,
                    state<_> + event<PowerOff> / []{ CurrentHandler = InitLoop;InitFlag.InitNI = true;} = state<Init>,

                    state<_> + event<IntoChassisLead> / []{ CurrentHandler = ChassisLeadLoop;InitFlag.ChassisLeadNI = true;} = state<ChassisLead>,
                    state<_> + event<IntoGimbalLead> / []{ CurrentHandler = GimbalLeadLoop;InitFlag.GimbalLeadNI = true;} = state<GimbalLead>,
                    state<_> + event<IntoAutoAim> / []{ CurrentHandler = AutoAimLoop;InitFlag.AutoAimNI = true;} = state<AutoAim>,
                    state<_> + event<IntoAutoRotate> / []{ CurrentHandler = AutoRotateLoop;InitFlag.AutoRotateNI = true;} = state<AutoRotate>,
                    state<_> + event<IntoIdle> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>,

                    state<_> + event<PausePress> / []{ CurrentHandler = IdleLoop;InitFlag.IdleNI = true;} = state<Idle>
            );
        }
    };

}


#endif
