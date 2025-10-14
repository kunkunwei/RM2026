#include "i2c.h"
#include "stm32f4xx_hal.h"

/**I2C3   
PA8     ------> I2C_SCL  
PC9     ------> I2C_SDA
*/

/*引脚配置 - 转换为HAL库*/
#define I2C_W_SCL(x)		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define I2C_W_SDA(x)		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define I2C_SDA_READ()  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9)

void I2C_delay(void)
{
    uint8_t i;
    for (i = 0; i < 10; i++);
}

/*引脚初始化*/
void I2C_Software_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOA和GPIOC时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // 配置SCL引脚 - PA8
    GPIO_InitStructure.Pin = GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置SDA引脚 - PC9
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 初始状态
    I2C_W_SCL(1);
    I2C_W_SDA(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void I2C_Start(void)
{
    I2C_W_SDA(1);
    I2C_W_SCL(1);
    I2C_delay();
    I2C_W_SDA(0);
    I2C_delay();
    I2C_W_SCL(0);
    I2C_delay();
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void I2C_Stop(void)
{
    I2C_W_SCL(0);
    I2C_W_SDA(0);
    I2C_W_SCL(1);
    I2C_delay();
    I2C_W_SDA(1);
}

void I2C_Ack(void)
{
    I2C_W_SDA(0);
    I2C_delay();
    I2C_W_SCL(1);
    I2C_delay();
    I2C_W_SCL(0);
    I2C_delay();
    I2C_W_SDA(1);
}

void I2C_NoAck(void)
{
    I2C_W_SCL(0);
    I2C_delay();
    I2C_W_SDA(1);
    I2C_delay();
    I2C_W_SCL(1);
    I2C_delay();
    I2C_W_SCL(0);
    I2C_delay();
}

uint8_t I2C_WaitAck(void)//0 正确应答；1 无应答
{
    I2C_W_SCL(0);
    I2C_W_SDA(1);
    I2C_delay();
    I2C_W_SCL(1);
    I2C_delay();
    if(I2C_SDA_READ())
    {
      I2C_W_SCL(0);
      I2C_delay();
      return 1;
    }
    else
    {
      I2C_W_SCL(0);
      I2C_delay();
      return 0;
  }
}

void I2C_SendByte(uint8_t Byte)
{
    uint8_t i;
    I2C_W_SCL(0);
    for (i = 0; i < 8; i++)
    {
      I2C_W_SDA(Byte & (0x80 >> i));
      I2C_delay();
      I2C_W_SCL(1);
      I2C_delay();
      I2C_W_SCL(0);
    }
    I2C_W_SDA(1);//8位数据发送完后，主机释放总线，以检测应答
}

uint8_t I2C_ReadByte(void)
{
    uint8_t i;
    uint8_t receiveByte = 0;

    for(i = 0; i < 8; i++)
    {
      receiveByte <<= 1;
      I2C_W_SCL(0);
      I2C_delay();
      I2C_W_SCL(1);
      I2C_delay();
      if(I2C_SDA_READ())
      {
        receiveByte++;
      }
    }
    I2C_W_SCL(0);

    return receiveByte;
}

//单字节写入
void I2C_Single_Write(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)		
{
      I2C_Start();
      I2C_SendByte(SlaveAddress);//发送设备地址+写信号
      I2C_WaitAck();
      I2C_SendByte(REG_Address);   //设置低起始地址      
      I2C_WaitAck();	
      I2C_SendByte(REG_data);
      I2C_WaitAck();   
      I2C_Stop(); 
}

//单字节读取
void I2C_Single_Read(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t* REG_data)
{   
    I2C_Start();
    I2C_delay();
    I2C_SendByte(SlaveAddress);
    I2C_WaitAck();

    I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress + 1);
    I2C_WaitAck();

    *REG_data = I2C_ReadByte();
    I2C_NoAck();
    I2C_Stop();
}
