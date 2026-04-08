#ifndef F407_RM_TMPLATE_HAL_BMI088_H
#define F407_RM_TMPLATE_HAL_BMI088_H

#include "main_cpp.h"

namespace Device{

    class BMI088{
    public:

        typedef enum
        {
            NO_ERROR                     = 0x00,
            ACC_PWR_CTRL_ERROR           = 0x01,
            ACC_PWR_CONF_ERROR           = 0x02,
            ACC_CONF_ERROR               = 0x03,
            ACC_SELF_TEST_ERROR          = 0x04,
            ACC_RANGE_ERROR              = 0x05,
            INT1_IO_CTRL_ERROR           = 0x06,
            INT_MAP_DATA_ERROR           = 0x07,
            GYRO_RANGE_ERROR             = 0x08,
            GYRO_BANDWIDTH_ERROR         = 0x09,
            GYRO_LPM1_ERROR              = 0x0A,
            GYRO_CTRL_ERROR              = 0x0B,
            GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
            GYRO_INT3_INT4_IO_MAP_ERROR  = 0x0D,
            SELF_TEST_ACCEL_ERROR        = 0x80,
            SELF_TEST_GYRO_ERROR         = 0x40,
            NO_SENSOR                    = 0xFF,
        }Status_e;

        struct Measurement{
            float acc[3]; // m/s²
            float gyro[3]; // rad/s
            float temp;    // °C
        };

        static BMI088& getInstance();

        void Init();
        void Update();
        Measurement GetMeasurement();
        void GetMeasurement(float *acc, float *gyro, float *temp);

    private:
        typedef enum {
            Accel,
            Gyro
        } Sensor_Type;

        struct Raw_data{
            int16_t acc[3];
            int16_t gyro[3];
            int16_t temp;
        } __attribute__((packed));

        typedef struct {
            GPIO_TypeDef *CS_Port;
            uint16_t CS_Pin;
            Sensor_Type t;
        } Sensor_cfg;

        BMI088() = default;
        ~BMI088() = default;
        BMI088(const BMI088&) = delete;
        BMI088& operator=(const BMI088&) = delete;

        Raw_data readRawData();
        Measurement tmp_measurement{};

        static void writeReg(Sensor_cfg info, uint8_t reg, uint8_t data);
        static uint8_t readReg(Sensor_cfg info, uint8_t reg);
        static void readRegs(Sensor_cfg info, uint8_t reg, uint8_t* data, uint8_t len);

        static Status_e accelInit();
        static Status_e gyroInit();

        static constexpr float TEMP_FACTOR = 0.125f;
        static constexpr float TEMP_OFFSET = 23.0f;
        static constexpr uint32_t WRITE_ACCEL_REG_NUM = 6;
        static constexpr uint32_t WRITE_GYRO_REG_NUM = 6;
        static constexpr float ACCEL_SENSITIVITY = 6.0f / 32768.0f * 9.80665f; // ±6g
        static constexpr float GYRO_SENSITIVITY = 2000.0f / 32768.0f * (3.14159265358979323846f / 180.0f); // ±2000°/s

        static inline const Sensor_cfg Accel_cfg = {CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, Accel};
        static inline const Sensor_cfg Gyro_cfg = {CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, Gyro};
    };

}

#endif
