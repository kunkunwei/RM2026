#include "IST8310.h"

using namespace Device;

IST8310& IST8310::getInstance(){
    static IST8310 instance;
    return instance;
}

uint8_t IST8310::read_single_reg(uint8_t reg)
{
    uint8_t res;
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

void IST8310::read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void IST8310::write_single_reg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c3, IST8310_IIC_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

bool IST8310::Init() {

    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    bool device_ok = true;

    device_ok &= read_single_reg(IST8310_WHO_AM_I) == IST8310_WHO_AM_I_VALUE;
    HAL_Delay(1);

    for (auto writeNum : ist8310_write_reg_data_error)
    {
        write_single_reg(writeNum[0], writeNum[1]);
        HAL_Delay(1);
        auto res = read_single_reg(writeNum[0]);
        HAL_Delay(1);

        device_ok &= res == writeNum[1];
    }

    return device_ok;

}

void IST8310::Update(){
    uint8_t buf[6];
    int16_t temp_ist8310_data = 0;
    read_muli_reg(0x03, buf, 6);
    temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
    tmp_measurement.data[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
    tmp_measurement.data[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
    tmp_measurement.data[2] = MAG_SEN * temp_ist8310_data;
}

void IST8310::GetMeasurement(float *mag_data){
    mag_data[0] = tmp_measurement.data[0];
    mag_data[1] = tmp_measurement.data[1];
    mag_data[2] = tmp_measurement.data[2];
}

IST8310::MegMeasurement IST8310::GetMeasurement(){
    return tmp_measurement;
}

