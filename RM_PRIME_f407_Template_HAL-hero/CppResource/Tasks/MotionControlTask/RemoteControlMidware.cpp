#include "MotionControl.h"
#include "MotionFSM.h"
#include "ShootTask.h"

#define BITMASK(a,n) (( (a)&(1<<n) )>>n)

namespace RemoteContrlMidware{


    static fsm::sm<MotionFSM::motionTransition> motion_fsm;
    Referee &hreferee = Referee::getInstance();

    void setMODE_SW(uint8_t mode){
        if(mode == 0)
            motion_fsm.process_event(MotionFSM::IntoChassisLead{});
        else if(mode == 1)
            motion_fsm.process_event(MotionFSM::IntoGimbalLead{});
        else if(mode == 2)
            motion_fsm.process_event(MotionFSM::IntoAutoAim{});
    }

    VirtualRC_CMD DBus_input(){

        using namespace MotionFSM;

        DBus& hDBus = DBus::getInstance();

        auto sta = hDBus.getState();

        uint8_t linLeft = sta->s[1];
        uint8_t linRight = sta->s[0];
        static short last_sta;
        short sta_check = (short)(linLeft) << 8 | linRight;

        if(last_sta != sta_check){
            last_sta = sta_check;
            if(linLeft == 1)motion_fsm.process_event(IntoIdle{});
            else if(linLeft == 3)motion_fsm.process_event(IntoChassisLead{});
            else if(linLeft==2 && linRight==3)motion_fsm.process_event(IntoGimbalLead{});
            else if(linLeft==2 && linRight==1)motion_fsm.process_event(IntoAutoAim{});
            else if(linLeft==2 && linRight==2)motion_fsm.process_event(IntoAutoRotate{});
        }

        return {{sta->ch[0], sta->ch[1], sta->ch[2], sta->ch[3]}, false};
    }

    VirtualRC_CMD VT03_input(){

        using namespace MotionFSM;

        VT03& hVT03 = VT03::getInstance();

        static VT03::RCState last_sta;

        const VT03::RCState *sta = hVT03.getState();

        // 键鼠控制逻辑
        // Q键：切换小陀螺开关（AutoRotate模式）
        if(BITMASK(sta->key_code, 6) && !BITMASK(last_sta.key_code, 6))
        {
            if(motion_fsm.is(fsm::state<AutoRotate>))
                //motion_fsm.process_event(IntoIdle{});  // 如果已经在AutoRotate模式，切换到Idle
            motion_fsm.process_event(IntoGimbalLead{});
            else
                motion_fsm.process_event(IntoAutoRotate{});
        }

        // E键：切换无力模式的进入与退出（Idle模式）
        if(BITMASK(sta->key_code, 7) && !BITMASK(last_sta.key_code, 7))
        {
            if(motion_fsm.is(fsm::state<Idle>))
            {
                // 如果已经在Idle模式，根据mode_sw切换到相应模式
                setMODE_SW(sta->mode_sw);
            }
            else
            {
                motion_fsm.process_event(IntoIdle{});
            }
        }

        // Z键：进入底盘主导模式（ChassisLead）
        if(BITMASK(sta->key_code, 8) && !BITMASK(last_sta.key_code, 8))
        {
            motion_fsm.process_event(IntoChassisLead{});
        }

        // X键：进入云台主导模式（GimbalLead）
        if(BITMASK(sta->key_code, 9) && !BITMASK(last_sta.key_code, 9))
        {
            motion_fsm.process_event(IntoGimbalLead{});
        }

        // C键：进入自瞄模式（AutoAim）
        if(BITMASK(sta->key_code, 10) && !BITMASK(last_sta.key_code, 10))
        {
            motion_fsm.process_event(IntoAutoAim{});
        }

        // 键鼠在线时，忽视遥控器的mode_sw状态机切换
        // 只有当没有键鼠按键按下时，才允许mode_sw切换
        bool keyboard_active = (sta->key_code != 0) || sta->mouse_l || sta->mouse_r || sta->mouse_middle;
        
        if(!keyboard_active)
        {
            // 只有在没有键鼠活动时，才处理mode_sw切换
            if(sta->mode_sw != last_sta.mode_sw)
                setMODE_SW(sta->mode_sw);
        }

        // 保留原有的暂停键功能
        if(sta->pause && !last_sta.pause)
        {
            if(motion_fsm.is(fsm::state<Idle>))
            {
                if(!keyboard_active)  // 只有在没有键鼠活动时，才允许暂停键切换模式
                    setMODE_SW(sta->mode_sw);
            }
            else
            {
                motion_fsm.process_event(PausePress{});
            }
        }

        // 保留原有的fn_1键功能（小陀螺）
        if(sta->fn_1 && !last_sta.fn_1)
        {
            if(motion_fsm.is(fsm::state<AutoRotate>))
            {
                if(!keyboard_active)  // 只有在没有键鼠活动时，才允许fn_1切换模式
                    setMODE_SW(sta->mode_sw);
            }
            else
            {
                motion_fsm.process_event(IntoAutoRotate{});
            }
        }

        short k_move = 300;
        if(BITMASK(sta->key_code, 4)) k_move = 660;
        if(BITMASK(sta->key_code, 5)) k_move = 100;

        short v_x = sta->ch[3] + (BITMASK(sta->key_code, 3) - BITMASK(sta->key_code, 2))*k_move;
        short v_y = sta->ch[2] + (BITMASK(sta->key_code, 0) - BITMASK(sta->key_code, 1))*k_move;
        short v_omega = sta->ch[0] + sta->mouse_x * 3;
        short v_pitch = sta->ch[1] + sta->mouse_y * 3;

        last_sta = *sta;
        return {{
                v_omega,
                v_pitch,
                v_x,
                v_y},
                static_cast<bool>(sta->trigger | BITMASK(sta->key_code, 4))
        };
    }

    void RemoteContrlMidwareInit(){

    }

    VirtualRC_CMD RemoteContrlMidwareLoop(){
        using namespace MotionFSM;

        // 读取裁判系统电源状态
        bool gimbal_on = true;
        bool chassis_on = true;
        
        if(hreferee.RefereeExist()) {
            // 如果裁判系统存在，读取实际的电源状态
            gimbal_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_gimbal_output;
            chassis_on = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_chassis_output;
        }
        
        static bool last_gimbal_on = false;
        static bool last_chassis_on = false;
        
        // 检测电源状态变化
        bool gimbal_power_just_off = (last_gimbal_on && !gimbal_on);
        bool chassis_power_just_off = (last_chassis_on && !chassis_on);
        bool gimbal_power_just_on = (!last_gimbal_on && gimbal_on);
        bool chassis_power_just_on = (!last_chassis_on && chassis_on);
        
        // 更新上次状态
        last_gimbal_on = gimbal_on;
        last_chassis_on = chassis_on;
        
        // 通知电源状态（ChassisControl::NotifyPowerSate会在断电时自动清除PID积分器）
        GimbalControl::NotifyPowerSate(gimbal_on);
        ChassisControl::NotifyPowerSate(chassis_on);
        TriggerControl::NotifyPowerSate(gimbal_on); // 使用云台电源状态控制发射系统
        
        // 如果底盘电源刚刚关闭，清除底盘运动目标值
        if (chassis_power_just_off) {
            // 底盘断电时，需要清除运动目标值，防止重新上电时乱转
            // 这里可以添加额外的清理逻辑，比如重置运动状态机等
            // Debug::print("Chassis power just turned off! Clearing motion targets...\n");
            
            // 触发PowerOff事件，让状态机回到Init状态
            motion_fsm.process_event(PowerOff{});
        }
        
        // 如果电源刚刚开启，重置所有积分器
        if (gimbal_power_just_on || chassis_power_just_on) {
            // 这里可以添加其他需要重置的积分器
            // 例如：ShootTask中的积分器
            // Debug::print("Power just turned on! Resetting integrators...\n");
        }

        // 检查是否在Init状态且电源已恢复
        if(motion_fsm.is(fsm::state<Init>))
        {
            // 如果电源已恢复，触发InitComplete事件切换到Idle状态
            if(chassis_on || gimbal_on)
            {
                motion_fsm.process_event(InitComplete{});
            }
        }
        else {
            // 如果不在Init状态，检查电源是否断开
            if(!(chassis_on && gimbal_on))
                motion_fsm.process_event(PowerOff{});
        }

        // 返回遥控器输入（如果在Init状态且电源未恢复，返回零指令）
        if(motion_fsm.is(fsm::state<Init>))
        {
            return {{0,0,0,0}, false};
        }
        else
        {
            return VT03_input() + DBus_input();
        }

    }

}