#include "ist8310driver_middleWare.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "i2c.h"

/**
  * @brief          初始化ist8310的GPIO
  * @param[in]      none
  * @retval         none
  */
void ist8310_GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOG时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();

    //IST RESET  PG6
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);
}

/**
  * @brief          初始化ist8310的通信接口
  * @param[in]      none
  * @retval         none
  */
void ist8310_com_init(void)
{
    I2C_Software_Init();
}

/**
  * @brief          通过I2C读取ist8310的一个字节
  * @param[in]      寄存器地址
  * @retval         寄存器值
  */
uint8_t ist8310_IIC_read_single_reg(uint8_t reg)
{
    //static const uint16_t IIC_time = 2000;
    uint8_t reg_data;
    I2C_Single_Read(IST8310_IIC_ADDRESS << 1, reg, &reg_data);
    //ist8310_delay_us(IIC_time);
    return reg_data;
}

/**
  * @brief          通过I2C写入一个字节到ist8310的寄存器中
  * @param[in]      寄存器地址
  * @retval         写入值
  */
void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data)
{
    I2C_Single_Write(IST8310_IIC_ADDRESS << 1, reg, data);
}

/**
  * @brief          通过I2C读取IST8310的多个字节
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      读取字节数
  * @retval         none
  */
void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    while (len)
    {
        (*buf) = ist8310_IIC_read_single_reg(reg);
        reg++;
        buf++;
        len--;
    }
}
/**
  * @brief          通过I2C写入多个字节到IST8310的寄存器
  * @param[in]      寄存器开始地址
  * @param[out]     存取缓冲区
  * @param[in]      写入字节数
  * @retval         none
  */
void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    static const uint16_t IIC_time = 2000;
    while (len)
    {
        ist8310_IIC_write_single_reg(reg, (*data));
        reg++;
        data++;
        len--;
        ist8310_delay_us(IIC_time);
    }
}


void ist8310_delay_ms(uint16_t ms)
    HAL_Delay(ms);
    delay_ms(ms);

}
void ist8310_delay_us(uint16_t us)
    delay_us(us); // 使用我们已经移植的delay_us函数
    delay_us(us);
}

void ist8310_RST_H(void)
    HAL_GPIO_WritePin(IST8310_RST_GPIO_Port, IST8310_RST_Pin, GPIO_PIN_SET);
    GPIO_SetBits(IST8310_RST_GPIO_Port, IST8310_RST_Pin);

}
extern void ist8310_RST_L(void)
    HAL_GPIO_WritePin(IST8310_RST_GPIO_Port, IST8310_RST_Pin, GPIO_PIN_RESET);
    GPIO_ResetBits(IST8310_RST_GPIO_Port, IST8310_RST_Pin);
}
