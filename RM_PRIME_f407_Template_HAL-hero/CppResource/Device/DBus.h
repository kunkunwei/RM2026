#ifndef F407_RM_TMPLATE_HAL_DBUS_H
#define F407_RM_TMPLATE_HAL_DBUS_H

#include "main_cpp.h"

namespace Device {


    class DBus {
    public:

        static constexpr uint32_t DBUS_FRAME_LENGTH = 18;
        static constexpr uint32_t RC_CH_VALUE_OFFSET = 1024;

        struct RCState {
            int16_t ch[5];       // -660~660
            uint8_t s[2];       // 1 or 3
            int16_t mouse_x;    // -32768~32768
            int16_t mouse_y;    // -32768~32768
            int16_t mouse_z;    // -32768~32768
            uint8_t press_l;    // 0 or 1
            uint8_t press_r;    // 0 or 1
            uint16_t key_code;  // bit field
        } __attribute__((packed));

        static DBus &getInstance();
        //输入18字节数据解析
        void receiveMessage(uint8_t *pdata);
        const volatile RCState* getState() const;

    private:

        DBus();
        ~DBus() = default;
        DBus(const DBus&) = delete;
        DBus& operator=(const DBus&) = delete;

        RCState state[2]{};
        volatile uint32_t active_index;

    };
}

#endif
