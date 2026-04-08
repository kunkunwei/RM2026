#include "TaskList.h"
#include "Referee.h"
#include "VT03.h"
#include "ui.h"
#include "MotionControl.h"
#include "MotionFSM.h"
#include "ShootTask.h"

using namespace Device;


void ClientUI_Task(void const * argument){

    using namespace RefereeType;
    using namespace RefereeType;
    Referee &hreferee = Referee::getInstance();
    auto &hVT03 = VT03::getInstance();

    static bool last_G = false;
    static bool last_V = false;

    while (1){

        uint8_t rid = hreferee.getRefereeInfo<GameRobotState>().robot_id;
        ui_self_id = rid;

        bool G = BITMASK(hVT03.getState()->key_code, 10);
        bool V = BITMASK(hVT03.getState()->key_code, 14);
        if(G && !last_G)
        {
            ui_init_g_Ungroup();
        } else if(V && !last_V)
        {
            ui_remove_g_Ungroup();
        } else{

            ui_g_Ungroup_AIM->color = MotionFSM::AutoAimOn ? 2 : 1;
            ui_g_Ungroup_LAUNCH3508->color = ShootFSM::getIsFribOpened() ? 2 : 1;
            ui_g_Ungroup_AUTOROTATE->color = RemoteContrlMidware::isAutoRotate() ? 2 : 1;

            ui_g_Ungroup_EGY_NUM->number = (int)ChassisControl::GetCapRest();
            ui_g_Ungroup_EGY_NUM->color =  ChassisControl::GetSupEnable() ? 2 : 1;
            ui_update_g_Ungroup();
        }
        last_V = V;
        last_G = G;

        osDelay(50);
    }

}