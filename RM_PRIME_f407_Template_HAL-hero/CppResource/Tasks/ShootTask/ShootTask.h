#ifndef F407_RM_TMPLATE_HAL_SHOOTTASK_H
#define F407_RM_TMPLATE_HAL_SHOOTTASK_H

#include "TaskList.h"
#include "DJiMotorGroup.h"
#include "DBus.h"
#include "DeltaPID.h"
#include "VT03.h"

using namespace Device;
using namespace Component;

#define MOTOR_RPM_TO_SPEED          0.00545324260857041989782339471524f       // 2PI/60/(3591/187)
constexpr float MAX_Fib_CUR = 10000.0f;
constexpr float MAX_Tri_CUR = 16000.0f;
constexpr float MAX_TRIG_SPEED = 4000.f;
constexpr float FRIC_SPEED = 3500.f;
constexpr float Tri_BACKING_SPEED = 4000.f;
constexpr float Tri_BACKING_TIME = 200.0f;
constexpr uint32_t SHOOT_GAP_MS = (uint32_t)((36.f*60000.f)/(0.2727f*MAX_TRIG_SPEED)) + 10;
constexpr uint32_t TOPEN = 600;

// 新增：射击控制参数
constexpr uint32_t SHOOT_DEBOUNCE_MS = 20;      // 射击按键消抖时间（毫秒）
constexpr uint32_t SHOOT_INTERVAL_MS = 300;     // 射击最小间隔时间（毫秒）

// 拨弹盘角度控制参数
constexpr float PI_THIRD = 1.0471975511965976f;        // π/3，60度，拨弹盘6个凹槽
constexpr float PI_SIXTH = 0.5235987755982988f;        // π/6，30度，用于额外补偿角度
constexpr float ANGLE_TOLERANCE = 0.08f;               // 角度容差（约4.58度）


namespace TriggerControl{
    void setTrigSpeed(float target_speed);
    void AddStep();  // 添加单发发射函数
    void Loop();     // 触发控制循环
    void NotifyPowerSate(bool s);  // 电源状态通知
    void SetExtraForwardStartAngle(float angle);  // 设置额外正转起始角度
    void TriggerExtraForward();  // 手动触发额外正转
    float GetCurrentAngle();  // 获取当前角度（供外部调用）
}

namespace FribControl{
    void NotifyPowerSate(bool s);
    void setFribSpeed(float target_speed);
    float * getFribSpeed();     // 获取摩擦轮速度
}

namespace ShootFSM{

    void FSM_Reset(void);
    void ShootFSM_Button_Loop(bool b);
    bool getIsFribOpened();
    bool getIsZeroCross();
    void ShootFSM_Wheel_Loop(short tin);

}

#endif //F407_RM_TMPLATE_HAL_SHOOTTASK_H
