#include "MotionControl.h"
#include "AHRS_middleware.h"

using namespace MotionParameter;

namespace ChassisControl{

    constexpr float MAX_CUR = 16000.0f;

    DJiMotorGroup m3508Group_Chassis(&hcan1, 0x201, 0x200);
    SupCap supCap(&hcan1);

    static bool isPowerOn = false;

    DeltaPID Speed_PID[4] = {
            DeltaPID(2.0f, 0.03f, 0.05f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.03f, 0.05f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.03f, 0.05f, 0.0f, MAX_CUR, -MAX_CUR),
            DeltaPID(2.0f, 0.03f, 0.05f, 0.0f, MAX_CUR, -MAX_CUR)
    };

    constexpr void MotionCalMecanumForward(const MoveState& target_motion, float* motor_v_target) {
        motor_v_target[0] = target_motion.vx + target_motion.vy - target_motion.omega;
        motor_v_target[1] = -target_motion.vx + target_motion.vy - target_motion.omega;
        motor_v_target[2] = target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[3] = -target_motion.vx + target_motion.vy + target_motion.omega;
    }

    constexpr void MotionCalMecanumBackward(const float motor_v_target[4], MoveState* cla_motion) {

        cla_motion->vx = 0.25f * (motor_v_target[0] - motor_v_target[1] + motor_v_target[2] - motor_v_target[3]);
        cla_motion->vy = 0.25f * ( motor_v_target[0] + motor_v_target[1] + motor_v_target[2] + motor_v_target[3]);
        cla_motion->omega = -0.25f * ( motor_v_target[0] + motor_v_target[1] - motor_v_target[2] - motor_v_target[3]);
    }

    constexpr void MotionCalOmnidForward(const MoveState& target_motion, float* motor_v_target) {
        // 直接用代数表达式（等价于对 (vx,vy) 逆时针旋转 +90° 后的映射）
        motor_v_target[0] = -target_motion.vx + target_motion.vy + target_motion.omega;
        motor_v_target[1] = -target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[2] = target_motion.vx - target_motion.vy + target_motion.omega;
        motor_v_target[3] = target_motion.vx + target_motion.vy + target_motion.omega;
    }

    constexpr void MotionCalOmnidBackward(const float motor_v_target[4], MoveState* cla_motion){
        // 直接用代数逆解（不使用中间变量），保证与前向映射互为逆运算
        float m0 = motor_v_target[0];
        float m1 = motor_v_target[1];
        float m2 = motor_v_target[2];
        float m3 = motor_v_target[3];

        cla_motion->vx = -0.25f * (m0 + m1 - m2 - m3);
        cla_motion->vy = -0.25f * (m1 + m2 - m0 - m3);
        cla_motion->omega = 0.25f * (m0 + m1 + m2 + m3);
    }

    void NotifyPowerSate(bool s){
        if(isPowerOn && !s){
            for(auto & i : Speed_PID){
                i.Reset();
            }
        }
        isPowerOn = s;
    }

    float GetPower(){
        return supCap.getPower();
    }

    uint32_t GetCapRest(){
        return supCap.getRest();
    }

    void set_supcap(bool enable){
        supCap.SetState(enable);
    }

    //实测omega=1980时，实际转动速度2rad/s左右(英雄机器人)
    // omega=900时，实际转动速度约1rad/s(云台跟随)
    MoveState setMove(MoveState target_state){
        static uint32_t cnt=0;
        cnt++;
        if(!isPowerOn){
            short tc[] = {0,0,0,0};
            m3508Group_Chassis.setMotorCurrent(tc);
            return {{0,0,0}};
        }

        using namespace RefereeType;
        auto&  hreferee = Referee::getInstance();
        float power_buffer = 60.f;
        float power_limit = 80.f;
        bool power_on = true;

        if(hreferee.RefereeExist()){
             power_buffer = hreferee.getRefereeInfo<PowerHeatData>().chassis_power_buffer;
             power_limit = hreferee.getRefereeInfo<GameRobotState>().chassis_power_limit;
             power_on = hreferee.getRefereeInfo<GameRobotState>().power_management_chassis_output;
        }
        // // else{
        // if (cnt>=200)
        // {
        //     cnt=0;
            supCap.SetParameter(power_buffer, power_limit, power_on);
        // }
            // supCap.SetParameter(60.f, 40.f);
        // }
        //功率限制
        float Power_margin = power_limit+power_buffer-60.0f;
        Power_margin=Power_margin>0?Power_margin:0;
        float power = supCap.getPower();
        float ref_rate = power / Power_margin;
        float k_limt = 1.0f;
        if(ref_rate > .9f){
            if(ref_rate > 1.0f)
                k_limt = 0.f;
            else
                k_limt = 10.f - 10.f * ref_rate;
        }
        target_state.vx *= k_limt;
        target_state.vy *= k_limt;
        target_state.omega *= k_limt;

        MoveState measure_state{};
        float motor_speed[4] = {0,0,0,0};
        for (int i = 0; i < 4; ++i) {
            DJiMotorGroup::MotorState bf = m3508Group_Chassis.getMotorState(i);
            motor_speed[i] = bf.speed;
        }
        MotionCalMecanumBackward(motor_speed, &measure_state);


        MoveState dv{{target_state.vx - measure_state.vx,
                   target_state.vy - measure_state.vy,
                   target_state.omega - measure_state.omega}};

        float vsq = dv.vx * dv.vx + dv.vy * dv.vy;
        float k_ref = CHASSIS_MAX_ACCEL*T_SAMPLE*AHRS_invSqrt(vsq);
        float k_omega_ref = CHASSIS_MAX_ALPHA*T_SAMPLE / (dv.omega > 0 ? dv.omega : -dv.omega + 0.0001f);
        if (k_ref < 1.0f ) {
            dv.vx *= k_ref;
            dv.vy *= k_ref;
        }
        if(k_omega_ref < 1.0f){
            dv.omega *= k_omega_ref;
        }
        MoveState target_state_accel_limit{{measure_state.vx + dv.vx,
                                 measure_state.vy + dv.vy,
                                 measure_state.omega + dv.omega}};

        float motor_v_target[4] = {};
        MotionCalMecanumForward(target_state_accel_limit, motor_v_target);

        short tmp_cur[] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            Speed_PID[i].Run(motor_v_target[i], motor_speed[i]);
            tmp_cur[i] = (short)Speed_PID[i].Output;
            //tmp_cur[i] = 600;
        }

        m3508Group_Chassis.setMotorCurrent(tmp_cur);

        return measure_state;
    }
}
