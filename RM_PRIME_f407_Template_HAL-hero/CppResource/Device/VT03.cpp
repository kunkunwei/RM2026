#include "VT03.h"
#include "CRC_Check.h"

using namespace Device;

VT03::VT03():active_index(0){}

VT03& VT03::getInstance() {
    static VT03 instance;
    return instance;
}

const VT03::RCState *const VT03::getState() const {
    return &state[active_index];
}

void VT03::receiveMessage(uint8_t *pdata) {
    auto* tmp_data = reinterpret_cast<remote_data_t*>(pdata);
    
    // 检查帧头 - 使用与PC控制代码一致的帧头 0xA9 0x53
    if (tmp_data->sof_1 != 0xA9 || tmp_data->sof_2 != 0x53) {
        return; // 帧头错误
    }
    
    if(!Component::CRC_Check::CRC16_Verify(pdata, VT03_FRAME_LENGTH))
        return;

    uint32_t next_index = !active_index;
    state[next_index].ch[0] = (int16_t)(tmp_data->ch_0) - RC_CH_VALUE_OFFSET;
    state[next_index].ch[1] = (int16_t)(tmp_data->ch_1) - RC_CH_VALUE_OFFSET;
    state[next_index].ch[2] = (int16_t)(tmp_data->ch_2) - RC_CH_VALUE_OFFSET;
    state[next_index].ch[3] = (int16_t)(tmp_data->ch_3) - RC_CH_VALUE_OFFSET;
    state[next_index].wheel = (int16_t)(tmp_data->wheel) - RC_CH_VALUE_OFFSET;
    state[next_index].mode_sw = (uint8_t)(tmp_data->mode_sw);
    state[next_index].pause = (bool)(tmp_data->pause);
    state[next_index].fn_1 = (bool)(tmp_data->fn_1);
    state[next_index].fn_2 = (bool)(tmp_data->fn_2);
    state[next_index].trigger = (bool)(tmp_data->trigger);
    state[next_index].mouse_x = tmp_data->mouse_x;
    state[next_index].mouse_y = tmp_data->mouse_y;
    state[next_index].mouse_z = tmp_data->mouse_z;
    state[next_index].mouse_l = tmp_data->mouse_left;
    state[next_index].mouse_r = tmp_data->mouse_right;
    state[next_index].mouse_middle = tmp_data->mouse_middle;
    state[next_index].key_code = tmp_data->key;

    active_index = next_index;
}
