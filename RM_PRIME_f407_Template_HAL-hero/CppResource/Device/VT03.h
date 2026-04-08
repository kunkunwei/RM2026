#ifndef F407_RM_TMPLATE_HAL_VT03_H
#define F407_RM_TMPLATE_HAL_VT03_H


#include "main_cpp.h"

namespace Device {

    #define BITMASK(a,n) (( (a)&(1<<n) )>>n)

    class VT03 {
    public:
        static constexpr uint32_t VT03_FRAME_LENGTH = 21;

        struct RCState {
            int16_t ch[4];       // -660~660

            uint8_t mode_sw;

            bool pause;
            bool fn_1;
            bool fn_2;
            bool trigger;
            int16_t wheel;

            int16_t mouse_x;    // -32768~32768
            int16_t mouse_y;    // -32768~32768
            int16_t mouse_z;    // -32768~32768
            bool mouse_l;    // 0 or 1
            bool mouse_r;    // 0 or 1
            bool mouse_middle;
            uint16_t key_code;  // bit field

            RCState& operator=(const RCState* other)
            {
                ch[0] = other->ch[0];
                ch[1] = other->ch[1];
                ch[2] = other->ch[2];
                ch[3] = other->ch[3];


                mode_sw = other->mode_sw;

                pause = other->pause;
                fn_1 = other->fn_1;
                fn_2 = other->fn_2;
                trigger = other->trigger;
                wheel = other->wheel;

                mouse_x = other->mouse_x;
                mouse_y = other->mouse_y;
                mouse_z = other->mouse_z;
                mouse_l = other->mouse_l;
                mouse_r = other->mouse_r;
                mouse_middle = other->mouse_middle;
                key_code = other->key_code;

                return *this;
            }
        };

        static VT03 &getInstance();

        //输入21字节数据解析
        void receiveMessage(uint8_t *pdata);
        const RCState *const getState() const;


    private:
        static constexpr uint32_t RC_CH_VALUE_OFFSET = 1024;

        struct remote_data_t
        {
            uint8_t sof_1;
            uint8_t sof_2;
            uint64_t ch_0:11;
            uint64_t ch_1:11;
            uint64_t ch_2:11;
            uint64_t ch_3:11;
            uint64_t mode_sw:2;
            uint64_t pause:1;
            uint64_t fn_1:1;
            uint64_t fn_2:1;
            uint64_t wheel:11;
            uint64_t trigger:1;

            int16_t mouse_x;
            int16_t mouse_y;
            int16_t mouse_z;
            uint8_t mouse_left:2;
            uint8_t mouse_right:2;
            uint8_t mouse_middle:2;
            uint16_t key;
            uint16_t crc16;
        } __attribute__ ((packed));


        VT03();
        ~VT03() = default;
        VT03(const VT03&) = delete;
        VT03& operator=(const VT03&) = delete;

        RCState state[2]{};
        volatile uint32_t active_index;

    };
}

#endif