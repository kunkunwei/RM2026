#include "ShootTask.h"
#include <cmath>
#include "Debug.h"  // 添加调试头文件

namespace TriggerControl{

    constexpr float i_tor_limit = 16000.f;

    uint32_t timeout = 0;
    int dir = -1;
    constexpr uint32_t trig_id = 2;
    bool isbacking = false;
    bool should_shoot = false;      // 单发发射标志
    bool shoot_ok=false;        // 发射完成标志
    uint32_t shoot_duration = 0;    // 发射持续时间计数器
    uint32_t shoot_start = 0;    // 发射开始时间计数器
    constexpr uint32_t SHOOT_DURATION_MS = 80; // 发射持续时间（毫秒）
    
    // 固定角度额外正转控制
    bool extra_forward_needed = false;  // 需要额外正转标志
    float start_angle = 0.0f; // 起始角度
    constexpr float EXTRA_FORWARD_ANGLE = 0.5235987755982988f; // π/6，30度，额外补偿角度
    constexpr float ANGLE_TOLERANCE = 0.05f; // 角度容差（约2.86度）
    // 卡弹检测：如果目标速度大于0但实际速度很低，可能是卡弹
    constexpr float BLOCK_TRIGGER_SPEED = 2*183.0f; // 卡弹触发速度阈值（rpm）
    float last_error = 0;
    float integral = 0;
    
    // 电源状态控制
    static bool isPowerOn = true;  // 电源状态（默认为开启）
    
    // 新增：定时反转控制
    uint32_t backing_start = 0;    // 反转开始时间
    constexpr uint32_t BACKING_DURATION_MS = 150; // 反转持续时间（150ms）
    constexpr uint32_t FORWARD_DURATION_MS = 100; // 正转持续时间（110ms）
    uint32_t forward_start = 0;    // 正转开始时间
    bool is_forwarding = false;    // 是否在正转中
    float backing_original_angle = 0.0f; // 反转开始时的原始角度

    DJiMotorGroup m3508Group_triger(&hcan1, 0x205, 0x1ff);

    // 角度跟踪变量
    static float current_angle = 0.0f;  // 当前角度（弧度）
    static int32_t ecd_count = 0;       // 编码器圈数计数
    static uint16_t last_ecd = 0;       // 上一次编码器值
    
    // 角度计算参数（根据你提供的宏定义）
    constexpr float MOTOR_ECD_TO_ANGLE = 0.000019970370880995190055505595881022f; // PI / (8192*3591/187)
    constexpr int32_t ECD_RANGE = 8191;
    constexpr int32_t HALF_ECD_RANGE = 4096;
    constexpr int32_t FULL_COUNT = 1975; // 3591/2

    // 获取拨弹盘当前角度
    float getTriggerAngle() {
        return current_angle;
    }

    // 角度标准化函数，将角度限制在[-π, π]范围内
    float rad_format(float angle) {
        constexpr float TWO_PI = 2.0f * M_PI;
        while (angle > M_PI) angle -= TWO_PI;
        while (angle < -M_PI) angle += TWO_PI;
        return angle;
    }

    // 计算角度差（考虑角度环绕）
    float angle_difference(float target, float current) {
        float diff = target - current;
        return rad_format(diff);
    }

    // 更新角度反馈（需要在主循环中调用）
    void UpdateAngleFeedback() {
        uint16_t current_ecd = m3508Group_triger.getMotorState(trig_id).angle;
        
        // 编码器丢帧检测
        int32_t ecd_diff = (int32_t)current_ecd - (int32_t)last_ecd;
        if (ecd_diff > HALF_ECD_RANGE) {
            ecd_count--;
        } else if (ecd_diff < -HALF_ECD_RANGE) {
            ecd_count++;
        }
        last_ecd = current_ecd;
        
        // 限制圈数计数范围
        if (ecd_count == FULL_COUNT) {
            ecd_count = -FULL_COUNT;
        } else if (ecd_count == -FULL_COUNT) {
            ecd_count = FULL_COUNT;
        }
        
        // 计算输出轴角度
        int32_t total_ecd = ecd_count * ECD_RANGE + (int32_t)current_ecd;
        current_angle = rad_format((float)total_ecd * MOTOR_ECD_TO_ANGLE);
    }

    void setTrigSpeed(float target_speed){
        // 更新角度反馈
        UpdateAngleFeedback();
        short tmp_cur[] = {0, 0, 0, 0};
        float motor_speed = m3508Group_triger.getMotorState(trig_id).speed;
        
        // 简化逻辑：定时反转 > 正常发射 > 空闲
        if (isbacking) {
            // 定时反转模式：反转固定时间
            uint32_t current_tick = osKernelSysTick();
            uint32_t backing_elapsed = current_tick - backing_start;
            
            if (backing_elapsed < BACKING_DURATION_MS) {
                // 反转阶段：持续反转200ms
                target_speed = -MAX_TRIG_SPEED * 0.4; // 反转速度降低
            } else {
                // 反转结束，开始正转
                target_speed = 0.0f;
                isbacking = false;
                is_forwarding = true;
                forward_start = osKernelSysTick();
                // 记录反转结束时的角度
                backing_original_angle = current_angle;
            }
        }
        else if (is_forwarding) {
            // 定时正转模式：正转固定时间
            uint32_t current_tick = osKernelSysTick();
            uint32_t forward_elapsed = current_tick - forward_start;
            
            if (forward_elapsed < FORWARD_DURATION_MS) {
                // 正转阶段：持续正转300ms
                target_speed = MAX_TRIG_SPEED;
            } else {
                // 正转结束
                target_speed = 0.0f;
                is_forwarding = false;
                shoot_ok = true;
                should_shoot = false; // 清除发射标志

            }
        }
        else if (should_shoot) {
            // 正常发射：角度控制
            float shoot_angle_diff = angle_difference(start_angle + PI_THIRD*0.8, current_angle);
            if (fabs(shoot_angle_diff) > ANGLE_TOLERANCE) {
                target_speed = MAX_TRIG_SPEED;
                shoot_ok = false;
                
                // 卡弹检测：如果目标速度大于0但实际速度很低
                if (fabs(motor_speed) < BLOCK_TRIGGER_SPEED) {
                    uint32_t current_tick = osKernelSysTick();
                    uint32_t elapsed_time = current_tick - shoot_start;
                    
                    // 如果低速持续时间超过200ms，判定为卡弹
                    if (elapsed_time > 70) { // 200ms
                        isbacking = true;
                        integral = 0;  // 重置积分器
                        backing_start = osKernelSysTick();
                        backing_original_angle = current_angle;
                        should_shoot = false; // 停止正常发射
                    }
                }
            } else {
                target_speed = 0.0f;
                shoot_ok = true;
                should_shoot = false; // 发射完成
            }
        }
        else {
            // 空闲状态
            target_speed = 0.0f;
        }


        float error = dir*target_speed - motor_speed;
        integral += error;
        float i_tor = integral * 0.03f;

        if(i_tor > i_tor_limit)
            i_tor = i_tor_limit;
        else if(i_tor < -i_tor_limit)
            i_tor = -i_tor_limit;

        float exp_i = error * 2. + i_tor;

        if(exp_i > MAX_Tri_CUR)
            exp_i = MAX_Tri_CUR;
        else if(exp_i < -MAX_Tri_CUR)
            exp_i = -MAX_Tri_CUR;

        tmp_cur[trig_id] = exp_i;
        m3508Group_triger.setMotorCurrent(tmp_cur);
    }

    // 单发发射函数 - 触发一次发射
    void AddStep() {
        if (!should_shoot) {  // 防止重复触发
            should_shoot = true;
            shoot_duration = 0;
            shoot_start = osKernelSysTick(); // 记录发射开始时间
            // 设置发射起始角度
            start_angle = current_angle;
        }
    }

    // 触发控制循环 - 需要在主循环中调用
    void Loop() {
        if (should_shoot) {
            shoot_duration++;
            if (shoot_duration >= SHOOT_DURATION_MS) {
            // if (shoot_ok==true){
                should_shoot = false;
                shoot_duration = 0;
            }
        }
    }



    // 电源状态通知函数
    void NotifyPowerSate(bool s) {
        static bool last_power_state = isPowerOn;
        
        // 如果电源状态从关闭变为开启，重置积分器
        if (!last_power_state && s) {
            integral = 0;  // 重置积分器
            should_shoot = false;  // 停止发射
            isbacking = false;  // 停止反转
            is_forwarding = false;  // 停止正转
            Debug::print("TriggerControl: Power on, resetting integrators\n");
        }
        
        // 如果电源状态从开启变为关闭，立即停止电机
        if (last_power_state && !s) {
            integral = 0;  // 重置积分器
            should_shoot = false;  // 停止发射
            isbacking = false;  // 停止反转
            is_forwarding = false;  // 停止正转
            
            // 立即停止电机
            short tmp_cur[] = {0, 0, 0, 0};
            m3508Group_triger.setMotorCurrent(tmp_cur);
            Debug::print("TriggerControl: Power off, stopping motor\n");
        }
        
        last_power_state = s;
        isPowerOn = s;
    }

    // 获取当前角度（供外部调用）
    float GetCurrentAngle() {
        return current_angle;
    }



}
