#include "ShootTask.h"

namespace FribControl{
    static bool isPowerOn = false;
    float speed[4]={0,0,0,0};
    DJiMotorGroup m3508Group_frib(&hcan2, 0x201, 0x200);
    DeltaPID Frib_PID[] = {
        DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
        DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
        DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR),
        DeltaPID(2.0f, 0.05f, 0.0f, 0.0f, MAX_Fib_CUR, -MAX_Fib_CUR)
};

    void NotifyPowerSate(bool s){
        if(isPowerOn && !s){
            for(auto & i : Frib_PID){
                i.Reset();
            }
        }
        isPowerOn = s;
    }

    void setFribSpeed(float target_speed){

        float target_speed_arr[] = {target_speed, target_speed,-target_speed,-target_speed};

        short tmp_cur[] = {0, 0, 0, 0};
        for (int i = 0; i < 4; ++i) {
            float motor_speed = m3508Group_frib.getMotorState(i).speed;
            Frib_PID[i].Run(target_speed_arr[i], motor_speed);
            tmp_cur[i] = (short)Frib_PID[i].Output;
            //tmp_cur[i] = 600;
        }
        m3508Group_frib.setMotorCurrent(tmp_cur);
    }
    

  // 获取四个摩擦轮电机的实时转速值（通过指针返回）
    float* getFribSpeed() {
        speed[0] = m3508Group_frib.getMotorState(0).speed;
        speed[1] = m3508Group_frib.getMotorState(1).speed;
        speed[2] = m3508Group_frib.getMotorState(2).speed;
        speed[3] = m3508Group_frib.getMotorState(3).speed;

        return speed;
    }



}
