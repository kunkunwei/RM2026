#include "DM4310.h"

namespace Device{

    DM4310::DM4310(CAN_HandleTypeDef *hc, uint32_t motorid, float MAX_POS, float MAX_VEL, float MAX_TOR):
    InterfaceCAN(hc, 0x010+motorid, motorid),
    active_ind(0),max_pos(MAX_POS), max_vel(MAX_VEL), max_tor(MAX_TOR)
    {

    }


    void DM4310::init(){
        uint8_t enable_motor_cmd[] =
                {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
        uint8_t len = 0x08;
        sendMessage(enable_motor_cmd, len);
    }

    bool DM4310::receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len){

        uint32_t next_ind = !active_ind;

        state[next_ind].id = (uint8_t)(pdata[0] & 0x0f);
        state[next_ind].sta = (StateCode)(pdata[0] >> 4);
        auto pos_int = (uint16_t)(pdata[1] << 8 | pdata[2]);
        auto vel_int = (uint16_t)(pdata[3] << 4 | pdata[4] >> 4);
        auto tor_int = (uint16_t)(((pdata[4] & 0x0f) << 8) | pdata[5]);
        state[next_ind].T_Mos = (float)(pdata[6]);
        state[next_ind].T_Rotor = (float)(pdata[7]);

        state[next_ind].pos = uint_to_float(pos_int, -max_pos,  max_pos, 16);
        state[next_ind].vel = uint_to_float(vel_int, -max_vel, max_vel, 12);
        state[next_ind].tor = uint_to_float(tor_int, -max_tor, max_tor, 12);

        active_ind = next_ind;

        return true;
    }


    void DM4310::setMITcmd(float target_pos, float target_vel, float Kp, float Kd, float target_tor) {

        if(state[active_ind].sta == StateCode::DISABLED){
            init();
            return;
        }

        uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;

        pos_tmp = float_to_uint(target_pos,  -max_pos,  max_pos,  16);
        vel_tmp = float_to_uint(target_vel,  -max_vel,  max_vel,  12);
        kp_tmp  = float_to_uint(Kp,   0, 500, 12);
        kd_tmp  = float_to_uint(Kd,   0, 5, 12);
        tor_tmp = float_to_uint(target_tor, -max_tor,  max_tor,  12);

        uint8_t pdata[8] = {0};
        pdata[0] = (pos_tmp >> 8);
        pdata[1] = pos_tmp;
        pdata[2] = (vel_tmp >> 4);
        pdata[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
        pdata[4] = kp_tmp;
        pdata[5] = (kd_tmp >> 4);
        pdata[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
        pdata[7] = tor_tmp;

        uint8_t len = 8;
        sendMessage(pdata, len);
    }

    DM4310::MotorState DM4310::getMotorState() {
        return state[active_ind];
    }

    void DM4310::setPosZero() {

        uint8_t pdata[8] = {0};
        pdata[0] = 0xFF;
        pdata[1] = 0xFF;
        pdata[2] = 0xFF;
        pdata[3] = 0xFF;
        pdata[4] = 0xFF;
        pdata[5] = 0xFF;
        pdata[6] = 0xFF;
        pdata[7] = 0xFE;

        uint8_t len = 8;
        sendMessage(pdata, len);
    }

    int DM4310::float_to_uint(float x_float, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
    }
    float DM4310::uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

}