#include "BMI088.h"
#include "BMI088_Reg.h"

using namespace Device;

namespace Device {

    static uint8_t Accel_Register_ConfigurationData_ErrorStatus[6][3] = {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON,BMI088::ACC_PWR_CTRL_ERROR},/* Turn on accelerometer */
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE,BMI088::ACC_PWR_CONF_ERROR},/* Pause mode */
        {BMI088_ACC_CONF,     (BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set),BMI088::ACC_CONF_ERROR},/* Acceleration Configuration */
        {BMI088_ACC_RANGE,    BMI088_ACC_RANGE_6G,BMI088::ACC_RANGE_ERROR},/* Accelerometer setting range */
        {BMI088_INT1_IO_CTRL, (BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW),BMI088::INT1_IO_CTRL_ERROR}, /* INT1 Configuration input and output pin */
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT,BMI088::INT_MAP_DATA_ERROR}/* interrupt map pin */
    };

    static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[6][3] = {
        {BMI088_GYRO_RANGE,             BMI088_GYRO_2000,BMI088::GYRO_RANGE_ERROR},/* Angular rate and resolution */
        {BMI088_GYRO_BANDWIDTH,         (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set),BMI088::GYRO_BANDWIDTH_ERROR},/* Data Transfer Rate and Bandwidth Settings */
        {BMI088_GYRO_LPM1,              BMI088_GYRO_NORMAL_MODE, BMI088::GYRO_LPM1_ERROR},/* Power Mode Selection Register */
        {BMI088_GYRO_CTRL,              BMI088_DRDY_ON,BMI088::GYRO_CTRL_ERROR},/* Data Interrupt Trigger Register */
        {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW),BMI088::GYRO_INT3_INT4_IO_CONF_ERROR}, /* Interrupt Pin Trigger Register */
        {BMI088_GYRO_INT3_INT4_IO_MAP,  BMI088_GYRO_DRDY_IO_INT3,BMI088::GYRO_INT3_INT4_IO_MAP_ERROR}/* interrupt map register */
    };


    static uint8_t BMI088_Read_Write_Byte(uint8_t txdata) {
        uint8_t rxdata = 0;
        HAL_SPI_TransmitReceive(&hspi1, &txdata, &rxdata, 1, 1000);
        return rxdata;
    }


    BMI088 &BMI088::getInstance() {
        static BMI088 instance;
        return instance;
    }

    void BMI088::Init() {
        BMI088::Status_e status_accel = NO_ERROR;
        BMI088::Status_e status_gyro = NO_ERROR;

        do
        {
            status_gyro = gyroInit();
            status_accel = accelInit();
            HAL_Delay(2);
        }while((status_gyro!=NO_ERROR)||(status_accel!=NO_ERROR));
    }

     BMI088::Raw_data BMI088::readRawData(){

        BMI088::Raw_data tmp;
        uint8_t buf[8] = {0};

        readRegs(Accel_cfg, BMI088_ACCEL_XOUT_L, buf, 6);

         tmp.acc[0] = (int16_t)((buf[1] << 8) | buf[0]);
         tmp.acc[1] = (int16_t)((buf[3] << 8) | buf[2]);
         tmp.acc[2] = (int16_t)((buf[5] << 8) | buf[4]);

        readRegs(Accel_cfg, BMI088_TEMP_M, buf, 2);
        tmp.temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (tmp.temp > 1023) tmp.temp -= 2048;

        readRegs(Gyro_cfg, BMI088_GYRO_CHIP_ID, buf, 8);

        if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            tmp.gyro[0] = (int16_t)((buf[3] << 8) | buf[2]);
            tmp.gyro[1] = (int16_t)((buf[5] << 8) | buf[4]);
            tmp.gyro[2] = (int16_t)((buf[7] << 8) | buf[6]);
        }

        return tmp;
    }


    void BMI088::Update(){
        Raw_data raw = readRawData();
        tmp_measurement.acc[0] = (float)raw.acc[0] * BMI088::ACCEL_SENSITIVITY;
        tmp_measurement.acc[1] = (float)raw.acc[1] * BMI088::ACCEL_SENSITIVITY;
        tmp_measurement.acc[2] = (float)raw.acc[2] * BMI088::ACCEL_SENSITIVITY;
        tmp_measurement.gyro[0] = (float)raw.gyro[0] * BMI088::GYRO_SENSITIVITY;
        tmp_measurement.gyro[1] = (float)raw.gyro[1] * BMI088::GYRO_SENSITIVITY;
        tmp_measurement.gyro[2] = (float)raw.gyro[2] * BMI088::GYRO_SENSITIVITY;
        tmp_measurement.temp = (float)raw.temp * BMI088::TEMP_FACTOR + BMI088::TEMP_OFFSET;
    }

    BMI088::Measurement BMI088::GetMeasurement() {
        return tmp_measurement;
    }

    void BMI088::GetMeasurement(float *acc, float *gyro, float *temp){
        for (int i = 0; i < 3; ++i) {
            acc[i] = tmp_measurement.acc[i];
            gyro[i] = tmp_measurement.gyro[i];
        }
        *temp = tmp_measurement.temp;
    }

    inline void BMI088::writeReg(Sensor_cfg info, uint8_t reg, uint8_t data){

        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_RESET);
        BMI088_Read_Write_Byte(reg);
        BMI088_Read_Write_Byte(data);
        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_SET);

    }

    inline uint8_t BMI088::readReg(Sensor_cfg info, uint8_t reg){

        uint8_t data;
        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_RESET);
        BMI088_Read_Write_Byte((reg) | 0x80);
        if(info.t == Accel) BMI088_Read_Write_Byte(0x55);
        data = BMI088_Read_Write_Byte(0x55);
        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_SET);
        return data;
    }

    inline void BMI088::readRegs(Sensor_cfg info, uint8_t reg, uint8_t* data, uint8_t len){

        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_RESET);
        BMI088_Read_Write_Byte(reg | 0x80);
        if(info.t == Accel) BMI088_Read_Write_Byte(0x55);
        for (int i = 0; i < len; ++i) {
            data[i] = BMI088_Read_Write_Byte(0x55);
        }
        HAL_GPIO_WritePin(info.CS_Port, info.CS_Pin, GPIO_PIN_SET);
    }


    BMI088::Status_e BMI088::accelInit() {

        uint8_t res = 0;

        writeReg(Accel_cfg,BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
        HAL_Delay(80); //软件复位等待时间，最长80ms

        //读取加速度计ID，重复两次
        res = readReg(Accel_cfg, BMI088_ACC_CHIP_ID);
        HAL_Delay(1);
        res = readReg(Accel_cfg,BMI088_ACC_CHIP_ID);
        HAL_Delay(1);

        if (res != BMI088_ACC_CHIP_ID_VALUE) {
            return NO_SENSOR;
        }

        for (uint8_t i = 0; i < WRITE_ACCEL_REG_NUM; i++) {
            writeReg(Accel_cfg, Accel_Register_ConfigurationData_ErrorStatus[i][0],
                          Accel_Register_ConfigurationData_ErrorStatus[i][1]);
            HAL_Delay(1);

            res = readReg(Accel_cfg,Accel_Register_ConfigurationData_ErrorStatus[i][0]);
            HAL_Delay(1);

            /* check the configuration and return the specified error */
            if (res != Accel_Register_ConfigurationData_ErrorStatus[i][1]) {
                return (Status_e) Accel_Register_ConfigurationData_ErrorStatus[i][2];
            }
        }

        return NO_ERROR;
    }

    BMI088::Status_e BMI088::gyroInit() {

        uint8_t res = 0;

        writeReg(Gyro_cfg, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
        HAL_Delay(80);

        res = readReg(Gyro_cfg,BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);
        res = readReg(Gyro_cfg,BMI088_GYRO_CHIP_ID);
        HAL_Delay(1);

        if (res != BMI088_GYRO_CHIP_ID_VALUE)
        {
            return NO_SENSOR;
        }

        /* config the gyro sensor */
        for (uint8_t i = 0; i < WRITE_GYRO_REG_NUM; i++)
        {
            writeReg(Gyro_cfg,Gyro_Register_ConfigurationData_ErrorStatus[i][0], Gyro_Register_ConfigurationData_ErrorStatus[i][1]);
            HAL_Delay(1);

            res = readReg(Gyro_cfg,Gyro_Register_ConfigurationData_ErrorStatus[i][0]);
            HAL_Delay(1);

            /* check the configuration and return the specified error */
            if (res != Gyro_Register_ConfigurationData_ErrorStatus[i][1])
            {
                return (Status_e)Gyro_Register_ConfigurationData_ErrorStatus[i][2];
            }
        }

        return NO_ERROR;
    }


}
