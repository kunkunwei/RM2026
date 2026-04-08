#ifndef F407_RM_TMPLATE_HAL_SUPCAP_H
#define F407_RM_TMPLATE_HAL_SUPCAP_H

#include "InterfaceCAN.h"

namespace Device{

class SupCap: public BSP::InterfaceCAN{
    public:
        SupCap(CAN_HandleTypeDef *hc);

        void init() override;
        bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) override;

        float getPower() const;
        uint32_t getRest() const;

        void SetParameter(float energy, float power_limit, bool power_on = true);
        void SetState(bool enable);

        ~SupCap() override = default;

    private:
        union RxData{
            struct {
                float P_measured;
                uint8_t Rest_energy;
                uint8_t reserve1;
                uint8_t reserve2;
                uint8_t Err_code;
            };
            uint8_t datas[8];
        };

        union TxData{
            struct {
                float E_buffer;
                uint8_t Ref_Power_Limit;
                uint8_t disChargeEnable;
                uint8_t chassis_Power_on;
                uint8_t reserve3;
            };
            uint8_t datas[8];
        };

        RxData rxData;
        bool supcap_enable = false;


    };

}

#endif //F407_RM_TMPLATE_HAL_SUPCAP_H
