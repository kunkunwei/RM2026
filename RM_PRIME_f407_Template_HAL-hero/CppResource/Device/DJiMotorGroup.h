#ifndef F407_RM_TMPLATE_HAL_DJIMOTORGROUP_H
#define F407_RM_TMPLATE_HAL_DJIMOTORGROUP_H

#include "InterfaceCAN.h"

namespace Device {


    class DJiMotorGroup: public BSP::InterfaceCAN {
    public:

        struct MotorState {
            int16_t angle;
            int16_t speed;
            int16_t current;
            uint8_t temp;
        };

        DJiMotorGroup(CAN_HandleTypeDef *hc, uint32_t RxId, uint32_t TxId);
        void init() override;
        bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) override;
        bool isMatchId(uint32_t id) const override;

        const MotorState getMotorState(int index) const;
        void setMotorCurrent(int16_t* current);

        ~DJiMotorGroup() override = default;



    private:
        MotorState motors[2][4];
        volatile uint32_t activeGroup[4];

    };

}

#endif
