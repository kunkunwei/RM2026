#ifndef F407_RM_TMPLATE_HAL_NANOMSG_H
#define F407_RM_TMPLATE_HAL_NANOMSG_H

#include "main_cpp.h"
#include "INS_Device.h"
#include "etl/queue.h"

namespace Device{

    class NanoMsg {
    public:

        union Bint32_Union{
            uint8_t U8_Buff[4];
            float Float;
        };

        union Control_Flag_t{

            struct _control_flags {
                uint8_t auto_aim : 1;        // 自动瞄准标志
                uint8_t auto_shoot : 1;      // 自动射击标志
                uint8_t exit_auto_aim : 1;   // 退出自动瞄准标志
                uint8_t exit_auto_shoot : 1; // 退出自动射击标志
                uint8_t reserved : 4;        // 保留位
            } bits;
            uint8_t all_flags;

        };

        static NanoMsg& getInstance();

        void init();

        INS_Device::Vector3 getControlCmd();

        /// 将数据存入串口解析FIFO
        /// @param data ///数据指针
        /// @param len 数据长度
        /// @return 返回实际存入的数据长度
        uint32_t pushData(uint8_t* data, uint32_t len);
        void sendMsg(float yaw, float pitch);
        void writeMsg(float yaw, float pitch);
        void sendMsg();
        void ProcessData();

    private:
        static constexpr int DIR_YAW = -1;
        static constexpr int DIR_PITCH = 1;

        NanoMsg() = default;
        NanoMsg(const NanoMsg&) = delete;
        NanoMsg& operator=(const NanoMsg&) = delete;

        etl::queue<uint8_t, 256> DataFifo;

        float tmp_yaw, tmp_pitch;

        Bint32_Union yaw_Union{};
        Bint32_Union pitch_Union{};
        Bint32_Union Depth_Union{};
        Control_Flag_t controlFlag{};

    };

}

#endif
