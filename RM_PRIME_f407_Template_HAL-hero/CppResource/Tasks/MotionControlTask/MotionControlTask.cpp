#include "MotionControl.h"
#include "MotionFSM.h"

using namespace MotionFSM;

[[noreturn]] void MotionControlTask(void const * argument){

    INS_Device &hINS = INS_Device::getInstance();
    RemoteContrlMidware::RemoteContrlMidwareInit();

    osDelay(200);
    MotionFSM::StateLoopArg input_sta{
        ChassisControl::setMove({{0,0,0}}),
        GimbalControl::getYawState(),
        GimbalControl::getPithState(),
        0.f
    };

    while (true){

        auto CMD = RemoteContrlMidware::RemoteContrlMidwareLoop();

        input_sta.YawSta = GimbalControl::getYawState();
        input_sta.PithSta = GimbalControl::getPithState();

        auto output_sta = MotionFSM::CurrentHandler(&CMD, hINS, input_sta);

        GimbalControl::setYawRelative(output_sta.YawSta);
        GimbalControl::setPithRelative(output_sta.PithSta);

        ChassisControl::set_supcap(CMD.OD_key);
        input_sta.ChassisSta = ChassisControl::setMove(output_sta.ChassisSta);

        if(MotionParameter::GIMBAL_PITCH_LOWER > output_sta.taget_pitch) output_sta.taget_pitch = MotionParameter::GIMBAL_PITCH_LOWER;
        else if(MotionParameter::GIMBAL_PITCH_UPPER < output_sta.taget_pitch) output_sta.taget_pitch = MotionParameter::GIMBAL_PITCH_UPPER;
        else input_sta.taget_pitch = output_sta.taget_pitch;

        osDelay(2);
    }
}



[[noreturn]] void DebugTask(void const * argument){

    Referee &hreferee = Referee::getInstance();
    VT03 &hVT03 = VT03::getInstance();

    while (true){

        // 注释掉调试输出，避免与TrigerControl.cpp的调试数据冲突
        // float pack[8];
        // 
        // pack[0] = hreferee.getRefereeInfo<RefereeType::PowerHeatData>().chassis_power_buffer;
        // pack[1] = ChassisControl::GetPower();
        // pack[2] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
        // pack[3] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
        // pack[4] = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_shooter_output;
        // pack[5] = hVT03.getState()->wheel;
        // pack[6] = (float)ChassisControl::GetCapRest();
        // 
        // Debug::print_vofa(pack, 8);

        osDelay(50);
    }
}
