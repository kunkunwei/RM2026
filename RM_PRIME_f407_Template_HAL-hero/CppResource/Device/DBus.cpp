#include "DBus.h"
using namespace Device;


DBus::DBus():
active_index(0)
{

}

DBus& DBus::getInstance() {
    static DBus instance;
    return instance;
}

const volatile DBus::RCState* DBus::getState() const {
    return &state[active_index];
}

void DBus::receiveMessage(uint8_t *pdata) {

    uint32_t next_index = !active_index;
    state[next_index].ch[0] = (int16_t)((pdata[0] | (pdata[1] << 8)) & 0x07ff);        //Channel 0
    state[next_index].ch[1] = (int16_t)(((pdata[1] >> 3) | (pdata[2] << 5)) & 0x07ff); //Channel 1
    state[next_index].ch[2] = (int16_t)(((pdata[2] >> 6) | (pdata[3] << 2) | (pdata[4] << 10))&0x07ff);//Channel 2
    state[next_index].ch[3] = (int16_t)(((pdata[4] >> 1) | (pdata[5] << 7)) & 0x07ff); //Channel 3
    state[next_index].s[0] = ((pdata[5] >> 4) & 0x0003);                       //Switch right
    state[next_index].s[1] = ((pdata[5] >> 4) & 0x000C) >> 2;                  //Switch left
    state[next_index].mouse_x = (int16_t)(pdata[6] | (pdata[7] << 8));                    //Mouse X axis
    state[next_index].mouse_y = (int16_t)(pdata[8] | (pdata[9] << 8));                    //Mouse Y axis
    state[next_index].mouse_z = (int16_t)(pdata[10] | (pdata[11] << 8));                  //Mouse Z axis
    state[next_index].press_l = pdata[12];                                  //!< Mouse Left Is Press ?
    state[next_index].press_r = pdata[13];                                  //!< Mouse Right Is Press ?
    state[next_index].key_code = pdata[14] | (pdata[15] << 8);              //KeyBoard value
    state[next_index].ch[4] = (int16_t)(pdata[16] | (pdata[17] << 8));                 //wheel

    state[next_index].ch[0] -= RC_CH_VALUE_OFFSET;
    state[next_index].ch[1] -= RC_CH_VALUE_OFFSET;
    state[next_index].ch[2] -= RC_CH_VALUE_OFFSET;
    state[next_index].ch[3] -= RC_CH_VALUE_OFFSET;
    state[next_index].ch[4] -= RC_CH_VALUE_OFFSET;

    active_index = next_index;
}


