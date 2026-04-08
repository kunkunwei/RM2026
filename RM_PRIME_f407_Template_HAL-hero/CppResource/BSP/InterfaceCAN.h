#ifndef F407_RM_TMPLATE_HAL_INTERFACECAN_H
#define F407_RM_TMPLATE_HAL_INTERFACECAN_H
#include "main_cpp.h"


namespace BSP {

    class InterfaceCAN {
    public:
        InterfaceCAN(CAN_HandleTypeDef *hcan,uint32_t RxId,uint32_t TxId);
        bool sendMessage(uint8_t* data, uint8_t &len);

        virtual void init() = 0;
        virtual bool isMatchId(uint32_t id) const;
        virtual bool receiveMessage(uint32_t id, uint8_t *pdata, uint32_t len) = 0;
        virtual ~InterfaceCAN() = default;

        CAN_HandleTypeDef *hCAN;

    protected:
        uint32_t RxStdId;
        uint32_t TxStdId;

    };

    void registerCANDevice(InterfaceCAN *canDevice);
    void CAN_UserInit();
}

#endif //F407_RM_TMPLATE_HAL_INTERFACECAN_H
