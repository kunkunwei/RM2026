#ifndef F407_RM_TMPLATE_HAL_REFEREETYPE_H
#define F407_RM_TMPLATE_HAL_REFEREETYPE_H

#include "main_cpp.h"
#include <cstring>
#include <tuple>

namespace RefereeType
{
    enum RobotID
    {
        RED_HERO = 1,
        RED_ENGINEER = 2,
        RED_STANDARD_1 = 3,
        RED_STANDARD_2 = 4,
        RED_STANDARD_3 = 5,
        RED_AERIAL = 6,
        RED_SENTRY = 7,
        BLUE_HERO = 11,
        BLUE_ENGINEER = 12,
        BLUE_STANDARD_1 = 13,
        BLUE_STANDARD_2 = 14,
        BLUE_STANDARD_3 = 15,
        BLUE_AERIAL = 16,
        BLUE_SENTRY = 17,
    };

    enum GameProgress
    {
        PROGRESS_UNSTART = 0,
        PROGRESS_PREPARE = 1,
        PROGRESS_SELFCHECK = 2,
        PROGRESS_5sCOUNTDOWN = 3,
        PROGRESS_BATTLE = 4,
        PROGRESS_CALCULATING = 5,
    };

    template<uint16_t ID>
    struct RefereeCMD{
        static constexpr uint16_t CMD_ID = ID;
    };

    struct GameState: RefereeCMD<0x0001> //0x0001
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    }__attribute__ ((packed));

    struct GameResult: RefereeCMD<0x0002> //0x0002
    {
        uint8_t winner;
    }__attribute__ ((packed));

    struct GameRobotHP: RefereeCMD<0x0003> // 0x0003
    {
        uint16_t red_1_robot_HP;
        uint16_t red_2_robot_HP;
        uint16_t red_3_robot_HP;
        uint16_t red_4_robot_HP;
        uint16_t reserved1;
        uint16_t red_7_robot_HP;
        uint16_t red_outpost_HP;
        uint16_t red_base_HP;
        uint16_t blue_1_robot_HP;
        uint16_t blue_2_robot_HP;
        uint16_t blue_3_robot_HP;
        uint16_t blue_4_robot_HP;
        uint16_t reserved2;
        uint16_t blue_7_robot_HP;
        uint16_t blue_outpost_HP;
        uint16_t blue_base_HP;
    } __attribute__ ((packed));

    struct EventData : RefereeCMD<0x0101> // 0x0101
    {
        uint32_t event_type;
    } __attribute__ ((packed));

    struct SupplyProjectileAction : RefereeCMD<0x0102> // 0x0102
    {
        uint8_t reserved;
        uint8_t supply_robot_id;
        uint8_t supply_projectile_step;
        uint8_t supply_projectile_num;
    } __attribute__ ((packed));

    struct RefereeWarning : RefereeCMD<0x0104> // 0x0104
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    } __attribute__ ((packed));

    struct GameRobotState : RefereeCMD<0x0201> // 0x0201
    {
        uint8_t robot_id;
        uint8_t robot_level;
        uint16_t current_HP;
        uint16_t maximum_HP;
        uint16_t shooter_barrel_cooling_value;
        uint16_t shooter_barrel_heat_limit;
        uint16_t chassis_power_limit;
        uint8_t power_management_gimbal_output : 1;
        uint8_t power_management_chassis_output : 1;
        uint8_t power_management_shooter_output : 1;
    } __attribute__ ((packed));

    struct PowerHeatData : RefereeCMD<0x0202> // 0x0202
    {
        uint16_t reserved1;
        uint16_t reserved2;
        float reserved;
        uint16_t chassis_power_buffer;
        uint16_t shooter_id1_17mm_cooling_heat;
        uint16_t shooter_id2_17mm_cooling_heat;
        uint16_t shooter_id1_42mm_cooling_heat;
    } __attribute__ ((packed));

    struct GameRobotPos : RefereeCMD<0x0203> // 0x0203
    {
        float x;
        float y;
        float angle;
    } __attribute__ ((packed));

    struct BuffMusk : RefereeCMD<0x0204> // 0x0204
    {
        uint8_t recovery_buff;
        uint8_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
    } __attribute__ ((packed));

    struct AerialRobotEnergy : RefereeCMD<0x0205> // 0x0205
    {
        uint8_t airforce_status;
        uint8_t time_remain;
    } __attribute__ ((packed));

    struct RobotHurt : RefereeCMD<0x0206> // 0x0206
    {
        uint8_t armor_type : 4;
        uint8_t hurt_type : 4;
    } __attribute__ ((packed));

    struct ShootData : RefereeCMD<0x0207> // 0x0207
    {
        uint8_t bullet_type;
        uint8_t shooter_number;
        uint8_t bullet_freq;
        float bullet_speed;
    } __attribute__ ((packed));

    struct BulletRemaining : RefereeCMD<0x0208> // 0x0208
    {
        uint16_t projectile_allowance_17mm;
        uint16_t projectile_allowance_42mm;
        uint16_t remaining_gold_coin;
    } __attribute__ ((packed));

    struct RFIDStatus : RefereeCMD<0x0209> // 0x0209
    {
        uint32_t rfid_status;
    } __attribute__ ((packed));

    struct GroundRobotPosition : RefereeCMD<0x020B> // 0x020B
    {
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float reserved1;
        float reserved2;
    } __attribute__ ((packed));

    struct RadarMarkData : RefereeCMD<0x020C> // 0x020C
    {
        uint8_t mark_progress;
    } __attribute__ ((packed));

    struct SentryInfo : RefereeCMD<0x020D> // 0x020D
    {
        uint32_t sentry_info;
    } __attribute__ ((packed));

    struct RadarInfo : RefereeCMD<0x020E> // 0x020E
    {
        uint8_t radar_info;
    } __attribute__ ((packed));

    struct StudentInteractiveData : RefereeCMD<0x0301> // 0x0301
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[1];
    } __attribute__ ((packed));

    struct MapData : RefereeCMD<0x0307> // 0x0307
    {
        uint8_t intention;
        uint16_t start_position_x;
        uint16_t start_position_y;
        int8_t delta_x[49];
        int8_t delta_y[49];
        uint16_t sender_id;
    } __attribute__ ((packed));

    struct CustomClientData : RefereeCMD<0x0306> // 0x0306
    {
        uint16_t key_value;
        uint16_t x_position : 12;
        uint16_t mouse_left : 4;
        uint16_t y_position : 12;
        uint16_t mouse_right : 4;
        uint16_t reserved;
    } __attribute__ ((packed));

    struct CustomInfo : RefereeCMD<0x0308> // 0x0308
    {
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[30];
    } __attribute__ ((packed));

    using RefereeTupleType =  std::tuple<
                GameState,
                GameResult,
                GameRobotHP,
                EventData,
                SupplyProjectileAction,
                RefereeWarning,
                GameRobotState,
                PowerHeatData,
                GameRobotPos,
                BuffMusk,
                AerialRobotEnergy,
                RobotHurt,
                ShootData,
                BulletRemaining,
                RFIDStatus,
                GroundRobotPosition,
                RadarMarkData,
                SentryInfo,
                RadarInfo,
                StudentInteractiveData,
                MapData,
                CustomClientData,
                CustomInfo
            >;


    template <std::size_t I = 0>
    void packetWriteTemplate(uint16_t ID, uint8_t* data, uint16_t len, RefereeTupleType& tuple) {
        if constexpr (I < std::tuple_size_v<RefereeTupleType>) {
            using T = std::tuple_element_t<I, RefereeTupleType>;
            if constexpr (std::is_base_of_v<RefereeCMD<T::CMD_ID>, T>) {
                if (T::CMD_ID == ID) {
                    std::memcpy(&std::get<I>(tuple), data, std::min<uint16_t>(len, sizeof(T)));
                    return;
                }
            }
            packetWriteTemplate<I + 1>(ID, data, len, tuple);
        }
    }



}

#endif
