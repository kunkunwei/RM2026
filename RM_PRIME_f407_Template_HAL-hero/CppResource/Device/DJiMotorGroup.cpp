#include "DJiMotorGroup.h"

namespace Device{

    DJiMotorGroup::DJiMotorGroup(CAN_HandleTypeDef *hc, uint32_t RxId, uint32_t TxId):
    InterfaceCAN(hc,RxId,TxId){

        for(int i = 0; i < 4; ++i){

            activeGroup[i] = 0;
            motors[0][i].angle = 0;
            motors[0][i].speed = 0;
            motors[0][i].current = 0;
            motors[0][i].temp = 0;
            motors[1][i].angle = 2;
            motors[1][i].speed = 0;
            motors[1][i].current = 0;
            motors[1][i].temp = 0;
        }

    }

    void DJiMotorGroup::init() {}

    bool DJiMotorGroup::receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) {

        int motorIndex = id - RxStdId;
        if(motorIndex < 0 || motorIndex >= 4) return false;

        uint32_t nextGroup = !activeGroup[motorIndex];
        motors[nextGroup][motorIndex].angle = (int16_t)(pdata[0] << 8 | pdata[1]);
        motors[nextGroup][motorIndex].speed = (int16_t)(pdata[2] << 8 | pdata[3]);
        motors[nextGroup][motorIndex].current = (int16_t)(pdata[4] << 8 | pdata[5]);
        motors[nextGroup][motorIndex].temp = pdata[6];
        activeGroup[motorIndex] = nextGroup;

        return true;
    }

    const DJiMotorGroup::MotorState DJiMotorGroup::getMotorState(int index) const {
        return motors[activeGroup[index]][index];
    }

    void DJiMotorGroup::setMotorCurrent(int16_t* current){
        if(current == nullptr) return;
        uint8_t data[8];
        for(int i = 0; i < 4; ++i){
            data[i*2] = (current[i] >> 8) & 0xFF;
            data[i*2 + 1] = current[i] & 0xFF;
        }
        uint8_t len = 0x08;
        sendMessage(data, len);
    }

    __inline bool DJiMotorGroup::isMatchId(uint32_t id) const  {
        return (id >= RxStdId) && (id < RxStdId + 4);
    }


}