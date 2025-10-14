#include "can.h"
#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//PD0 CAN1_Rx、PD1 CAN1_Tx
uint8_t CAN1_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode)
{
    // 在HAL库中，CAN初始化已经在CubeMX生成的can.c中实现
    // 这里我们只需要配置过滤器和使能中断

    CAN_FilterTypeDef CAN_FilterInitStructure;

    // 配置CAN过滤器
    CAN_FilterInitStructure.FilterBank = 0;                     // 过滤器0
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK; // 屏蔽模式
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;// 过滤器位宽32位
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;              // 32位id
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;          // 32位mask
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;// 过滤器0关联到fifo0
    CAN_FilterInitStructure.FilterActivation = ENABLE;          // 激活过滤器
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;          // CAN1的过滤器范围：0-13

    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);

    // 启动CAN接收
    HAL_CAN_Start(&hcan1);

    // 使能CAN接收中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    return 0;
}

//PB5 CAN2_Rx、 PB6 CAN2_Tx
uint8_t CAN2_mode_init(uint8_t tsjw, uint8_t tbs2, uint8_t tbs1, uint16_t brp, uint8_t mode)
{
    CAN_FilterTypeDef CAN_FilterInitStructure;

    // 配置CAN过滤器
    CAN_FilterInitStructure.FilterBank = 14;                    // 过滤器14
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK; // 屏蔽模式
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;// 过滤器位宽32位
    CAN_FilterInitStructure.FilterIdHigh = 0x0000;              // 32位id
    CAN_FilterInitStructure.FilterIdLow = 0x0000;
    CAN_FilterInitStructure.FilterMaskIdHigh = 0x0000;          // 32位mask
    CAN_FilterInitStructure.FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_RX_FIFO0;// 过滤器14关联到fifo0
    CAN_FilterInitStructure.FilterActivation = ENABLE;          // 激活过滤器
    CAN_FilterInitStructure.SlaveStartFilterBank = 14;          // CAN2的过滤器从14开始

    HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterInitStructure);

    // 启动CAN接收
    HAL_CAN_Start(&hcan2);

    // 使能CAN接收中断
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

    return 0;
}
