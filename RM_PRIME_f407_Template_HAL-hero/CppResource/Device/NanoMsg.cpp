#include "NanoMsg.h"
#include "usbd_cdc_if.h"

using namespace Device;

NanoMsg &NanoMsg::getInstance(){
    static NanoMsg instance;
    return instance;
}

uint32_t NanoMsg::pushData(uint8_t* data, uint32_t len){

    if (DataFifo.available()<len) {
        len = DataFifo.available();
    }

    for (uint32_t i = 0; i < len; i++){
        DataFifo.push(data[i]);
    }

    return len;
}

void NanoMsg::sendMsg(float yaw, float pitch){

    uint8_t sendBuff[13];

    sendBuff[0] = 0x42; // 设置帧头
    sendBuff[1] = 0x21; // 设置地址
    sendBuff[2] = 13;   // 设置帧长

    float data_array[] = {DIR_YAW*yaw, DIR_PITCH*pitch};

    for (int j = 0; j < 2; j++)
    {
        uint8_t* float_data_ptr = (uint8_t*)&data_array[j];
        for (uint32_t i = 0; i < sizeof(float); i++)
        {
            sendBuff[3 + j * sizeof(float) + i] = float_data_ptr[i];
        }
    }
    sendBuff[11] = 0xFF;

    uint8_t check = 0x00;
    for(int i = 0; i < 12; i++)
        check += sendBuff[i];
    sendBuff[12] = check;

    CDC_Transmit_FS(sendBuff, 13);
}

INS_Device::Vector3 NanoMsg::getControlCmd(){
    INS_Device::Vector3 rel = {{{0.f, 0.f, 0.f}}};
    rel.yaw = DIR_YAW*yaw_Union.Float;
    rel.pitch = DIR_PITCH*pitch_Union.Float;

    return rel;
}

void NanoMsg::ProcessData(){

    while (DataFifo.front() != 0x42 && DataFifo.current_size > 17)
        DataFifo.pop();

    if(DataFifo.current_size >= 17 && DataFifo.front() == 0x42){
        uint8_t temp[17], check = 0;
        for(int i = 0; i < 17; i++){
            temp[i] = DataFifo.front();
            DataFifo.pop();
        }

        for (int i = 0; i < 15; i++) {
            check += temp[i];
        }
        check = temp[16];
        if(check == temp[16])
        {
            yaw_Union.U8_Buff[0] = temp[3];
            yaw_Union.U8_Buff[1] = temp[4];
            yaw_Union.U8_Buff[2] = temp[5];
            yaw_Union.U8_Buff[3] = temp[6];

            pitch_Union.U8_Buff[0] = temp[7];
            pitch_Union.U8_Buff[1] = temp[8];
            pitch_Union.U8_Buff[2] = temp[9];
            pitch_Union.U8_Buff[3] = temp[10];

            Depth_Union.U8_Buff[0] = temp[11];
            Depth_Union.U8_Buff[1] = temp[12];
            Depth_Union.U8_Buff[2] = temp[13];
            Depth_Union.U8_Buff[3] = temp[14];

            controlFlag.all_flags = temp[15];
        }

    }

}

void NanoMsg::writeMsg(float yaw, float pitch){
    tmp_yaw = yaw;
    tmp_pitch = pitch;
}

void NanoMsg::sendMsg(){
    sendMsg(tmp_yaw ,tmp_pitch);
}
