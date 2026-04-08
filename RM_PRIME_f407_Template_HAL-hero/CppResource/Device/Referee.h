#ifndef F407_RM_TMPLATE_HAL_REFEREE_H
#define F407_RM_TMPLATE_HAL_REFEREE_H

#include "main_cpp.h"
#include "etl/queue.h"
#include "RefereeType.h"
#include "boost/sml.hpp"
#include "CRC_Check.h"


namespace RefereeFSM{

    static constexpr uint8_t HEADER_SOF = 0xA5;
    static constexpr uint32_t FRAME_MAX_SIZE = 128;

    static constexpr uint32_t HEADER_SIZE = 5;
    static constexpr uint32_t CMD_SIZE = 2;
    static constexpr uint32_t CRC16_SIZE = 2;

    static constexpr uint32_t HEADER_CRC_CMDID_LEN = (HEADER_SIZE + CRC16_SIZE + CMD_SIZE);

    struct depackContext{
        uint16_t data_len = 0;
        uint16_t data_index = 0;
        union {
            struct {
                uint8_t sof;
                uint16_t data_length;
                uint8_t seq;
                uint8_t crc8;
                uint16_t cmd_id;
                uint8_t data[FRAME_MAX_SIZE - HEADER_CRC_CMDID_LEN];
                uint16_t crc16;
            } __attribute__ ((packed));
            uint8_t  protocol_packet[FRAME_MAX_SIZE]{};
        };
    };

    struct unitData {uint8_t byte;};

    struct isHead {
        bool operator()(const unitData& data) const {
            return data.byte == HEADER_SOF;
        }
    };
    struct CRC8_Pass {
        bool operator()(depackContext& context, const unitData& data) const {
            context.protocol_packet[4] = data.byte;
            context.data_index = 5;
            //return true;
            return Component::CRC_Check::CRC8_Verify(context.protocol_packet, 5);
        }
    };

    struct CRC16_Pass {
        bool operator()(depackContext& context, const unitData& data) const {

            if(context.data_index < FRAME_MAX_SIZE)
                context.protocol_packet[context.data_index++] = data.byte;

            if(context.data_len + HEADER_CRC_CMDID_LEN == context.data_index)
                return Component::CRC_Check::CRC16_Verify(context.protocol_packet, context.data_index);
                //return true;
            return false;
        }
    };

    struct PackChecked{};

    struct depackTransition{
        auto operator()() const{
            using namespace boost::sml;
            return make_transition_table(

                    *"HEAD_SOF"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    "LENGTH_LOW"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    "LENGTH_HIGH"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    "FRAME_SEQ"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    "HEADER_CRC8"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    "DATA_CRC16"_s + event<unitData> [isHead{}] = "LENGTH_LOW"_s,
                    state<PackChecked> + event<unitData> [isHead{}] = "LENGTH_LOW"_s,

                    "LENGTH_LOW"_s + event<unitData> /
                    [](depackContext& context, const unitData& data){
                        context.data_len = data.byte;
                        context.protocol_packet[0] = HEADER_SOF;
                        context.protocol_packet[1] = data.byte;
                    }  = "LENGTH_HIGH"_s,

                    "LENGTH_HIGH"_s + event<unitData> /
                    [](depackContext& context, const unitData& data){
                        context.data_len |= (data.byte << 8);
                        context.protocol_packet[2] = data.byte;
                    }  = "FRAME_SEQ"_s,

                    "FRAME_SEQ"_s + event<unitData> /
                    [](depackContext& context, const unitData& data){
                        context.protocol_packet[3] = data.byte;
                    }  = "HEADER_CRC8"_s,

                    "HEADER_CRC8"_s + event<unitData> [CRC8_Pass{}] = "DATA_CRC16"_s,
                    "HEADER_CRC8"_s + event<unitData>  = "HEAD_SOF"_s,

                    "DATA_CRC16"_s + event<unitData> [CRC16_Pass{}] = state<PackChecked>

            );
        }
    };

}

namespace Device{

    class Referee {
    public:

        static constexpr uint32_t MIN_FRAME_SIZE = 9;
        static constexpr uint32_t MAX_TIME_COUNT = 500;


        static Referee& getInstance();

        void init();
//        void Send(uint8_t* data, uint8_t len);

        /// 将数据存入裁判系统串口解析FIFO
        /// @param data ///数据指针
        /// @param len 数据长度
        /// @return 返回实际存入的数据长度
        uint32_t pushData(uint8_t* data, uint32_t len);

        template<typename T>
        std::enable_if_t<std::is_base_of_v<RefereeType::RefereeCMD<T::CMD_ID>, T>,T&>
        getRefereeInfo() {
            return std::get<T>(RefereeTuple);
        }

        void ProcessData();
        bool RefereeExist() const;

    private:
        Referee() = default;
        Referee(const Referee&) = delete;
        Referee& operator=(const Referee&) = delete;

        RefereeFSM::depackContext depackCtx;
        boost::sml::sm<RefereeFSM::depackTransition> depackFSM{depackCtx};
        etl::queue<uint8_t, 256> DataFifo;
        RefereeType::RefereeTupleType RefereeTuple;
        uint32_t referee_time_out_count = MAX_TIME_COUNT;

        void packetWrite(uint16_t ID, uint8_t* data, uint16_t len);
    };

}

#endif
