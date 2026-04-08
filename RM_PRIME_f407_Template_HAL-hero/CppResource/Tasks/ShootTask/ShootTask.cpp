#include "ShootTask.h"

#include <cmath>

#include "Referee.h"

using namespace ShootFSM;
using namespace FribControl;
using namespace TriggerControl;
//#define USE_WHEEL_CTRL_

static uint32_t shootcount = 0;
static bool last_mouse_l = false;  // 上一次鼠标左键状态，用于检测单击
static bool last_trigger = false;  // 上一次扳机状态
static bool shoot_debounce = false; // 射击消抖标志
static uint32_t shoot_press_tick = 0; // 射击按键按下时间
static uint32_t last_shoot_time = 0;  // 上次射击时间

[[noreturn]] void ShootTask(void const * argument){

    VT03 &hVT03 = VT03::getInstance();
    // auto &hreferee = Referee::getInstance();  // 注释掉裁判系统引用
    while (true){
        // bool booster_power = hreferee.getRefereeInfo<RefereeType::GameRobotState>().power_management_shooter_output;
        // if(!booster_power) ShootFSM::FSM_Reset();
        // 直接设置发射电源为开启状态
        bool booster_power = true;
        if(!booster_power) ShootFSM::FSM_Reset();
#ifdef USE_WHEEL_CTRL_
        // 注意：USE_WHEEL_CTRL_模式下需要DBus实例
        DBus &hDbus = DBus::getInstance();
        short tin = -hDbus.getState()->ch[4] + hVT03.getState()->wheel;
        ShootFSMLoop(tin);

        bool isFribOpened = getIsFribOpened();
        bool isZeroCross = getIsZeroCross();

        float triger_speed, frib_speed;
        if(isFribOpened) {
            if(tin < 0) tin = 0;

            if(isZeroCross)
                triger_speed = 1.f*tin;
            else
                triger_speed = 0.f;

            frib_speed = 3500;
        }
        else{
            triger_speed = 0.f;
            frib_speed = 0.f;
            shootcount = SHOOT_GAP_MS;
        }

        setFribSpeed(frib_speed);
        setTrigSpeed(triger_speed);
#else

        auto vt_sta = hVT03.getState();
        uint32_t now_tick = osKernelSysTick();

        // 检测鼠标左键单击（上升沿检测）
        bool current_mouse_l = vt_sta->mouse_l;
        bool mouse_l_clicked = (current_mouse_l && !last_mouse_l);
        last_mouse_l = current_mouse_l;

        // 检测扳机状态
        bool current_trigger = vt_sta->trigger;
        bool trigger_clicked = (current_trigger && !last_trigger);
        last_trigger = current_trigger;

        // 射击触发条件：遥控器扳机或鼠标左键
        bool shoot_input = current_trigger || current_mouse_l;

        // 拨弹机构开关控制：FN2+扳机 或 鼠标中键
        bool b = (vt_sta->fn_2 && vt_sta->trigger) || vt_sta->mouse_middle;
        ShootFSM::ShootFSM_Button_Loop(b);

        bool isFribOpened = ShootFSM::getIsFribOpened();
        bool isZeroCross = ShootFSM::getIsZeroCross();
        float frib_speed = 0.f;
        float triger_speed = 0.f;  // 默认触发速度为0
        float *frib_speed_arr = getFribSpeed(); // 获取摩擦轮速度数组
        if(isFribOpened) {
            frib_speed = 3500.f;  //
            // 拨弹机构已打开，可以射击

            // 射击按键处理：添加消抖和间隔控制
            if ((trigger_clicked || mouse_l_clicked) && !shoot_debounce) {
                // 按键按下边沿
                shoot_debounce = true;
                shoot_press_tick = now_tick;
            }

            // 消抖处理：按键稳定后触发
            if (shoot_debounce && (now_tick - shoot_press_tick) >= SHOOT_DEBOUNCE_MS) {
                shoot_debounce = false; // 消抖完成
                // 检查射击间隔
                if ((now_tick - last_shoot_time) >= SHOOT_INTERVAL_MS) {
                    if (fabs(frib_speed_arr[0]) >FRIC_SPEED*0.92 && fabs(frib_speed_arr[0])  < FRIC_SPEED*1.02&&
                        fabs(frib_speed_arr[1]) > FRIC_SPEED*0.92 && fabs(frib_speed_arr[1]) < FRIC_SPEED*1.02&&
                        fabs(frib_speed_arr[2]) >FRIC_SPEED*0.92 && fabs(frib_speed_arr[2])  < FRIC_SPEED*1.02&&
                        fabs(frib_speed_arr[3]) > FRIC_SPEED*0.92 && fabs(frib_speed_arr[3]) < FRIC_SPEED*1.02) {

                        // 满足间隔条件，触发射击
                        if (isZeroCross) {
                            TriggerControl::AddStep();  // 触发一次发射
                            last_shoot_time = now_tick; // 更新上次射击时间
                        }
                    }
                    // // 满足间隔条件，触发射击
                    // if (isZeroCross) {
                    //     TriggerControl::AddStep();  // 触发一次发射
                    //     last_shoot_time = now_tick; // 更新上次射击时间
                    // }
                }
            }
            
            
        } else {
            // 拨弹机构关闭，重置计数器
            shootcount = SHOOT_GAP_MS;
            shoot_debounce = false; // 重置消抖标志
        }

        // 设置拨弹机构速度
        FribControl::setFribSpeed(frib_speed);

        // 设置触发速度（TriggerControl::setTrigSpeed会根据should_shoot标志自动调整速度）
        TriggerControl::setTrigSpeed(triger_speed);

        // 更新触发控制状态
        TriggerControl::Loop();
#endif


        osDelay(1);

    }

}
