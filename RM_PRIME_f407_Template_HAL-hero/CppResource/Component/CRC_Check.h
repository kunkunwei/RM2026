#ifndef F407_RM_TMPLATE_HAL_CRC_H
#define F407_RM_TMPLATE_HAL_CRC_H

#include "main_cpp.h"

namespace Component{

    class CRC_Check {

    public:
        static uint8_t CRC8_Cla(uint8_t* data, uint32_t len);
        static uint16_t CRC16_Cla(uint8_t* data, uint32_t len);

        //最后一个字节为CRC8
        static bool CRC8_Verify(uint8_t* data, uint32_t len);
        //最后两字节CRC16
        static bool CRC16_Verify(uint8_t* data, uint32_t len);

    };
}


#endif
