#include "spi.h"
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

/**SPI1 GPIO Configuration    
PB4     ------> SPI1_MISO
PB3     ------> SPI1_SCK
PA7     ------> SPI1_MOSI
*/
void SPI1_Init(void)
{
    // 在HAL库中，SPI和GPIO的初始化已经在CubeMX生成的代码中完成
    // 这里只需要启动SPI外设

    // 初始化SPI配置，低速模式用于IMU初始化
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    HAL_SPI_Init(&hspi1);
}

void SPI1_SetSpeedAndDataSize(uint16_t Speed, uint16_t DataSize)
{
    // 关闭SPI
    __HAL_SPI_DISABLE(&hspi1);

    // 更新SPI配置
    hspi1.Init.BaudRatePrescaler = Speed;

    if(DataSize == SPI_DATASIZE_8BIT)
    {
        hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    }
    else
    {
        hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
    }

    // 重新初始化SPI
    HAL_SPI_Init(&hspi1);

    // 启动SPI
    __HAL_SPI_ENABLE(&hspi1);
}

uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
    uint8_t RxData = 0;

    // 使用HAL库的SPI传输函数
    HAL_SPI_TransmitReceive(&hspi1, &TxData, &RxData, 1, HAL_MAX_DELAY);

    return RxData;
}
