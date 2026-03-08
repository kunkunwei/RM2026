/**
  * @file       imgtrans_control.c
  * @brief      图传（图像传输）串口模块，基于 USART1 + DMA 双缓冲接收。
  *
  * 本模块只负责：
  *   1. 初始化与 USART1 相关的 DMA 双缓冲接收；
  *   2. 在串口 / DMA 接收完成中断中，把数据搬运到本地缓冲区；
  *   3. 提供简单的发送接口，方便上层发送图像或控制数据；
  *
  */
#include <string.h>
#include "bsp_rc.h"          // 复用 myUSART_DMAEx_MultiBuffer_Init 声明
#include "crc8_crc16.h"      // CRC 校验
#include "imgtrans_control.h"




// USART1 句柄 & DMA 句柄在 usart.c / dma.c 中定义
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;

static void imgtrans_to_rc(volatile const uint8_t *imgtrans_buf, imgtrans_ctrl_t *imgtrans_ctrl);

__attribute__ ((section(".dma_buffer"), aligned(32)))       // RAM管理，非H7系列不需要
static uint8_t img_rx_dma_buf[2][VT_RX_BUF_SIZE];
// 图传控制结构体的静态实例
imgtrans_ctrl_t s_imgtrans_ctrl;

// 最近一次接收到的原始帧缓存，用于 CRC 校验
static uint8_t s_last_frame_buf[VT_FRAME_LENGTH];

/**
 * @brief 图传模块初始化。
 *
 * 当前实现：
 *  - 清空控制结构体；
 *  - 使用 myUSART_DMAEx_MultiBuffer_Init 配置 USART10 的 DMA 双缓冲接收；
 *  - 不在此处做协议解析，只负责把原始数据放入 s_imgtrans_ctrl.rx_buf。
 */
void imgtrans_init(void)
{
    memset(&s_imgtrans_ctrl, 0, sizeof(s_imgtrans_ctrl));
    memset(img_rx_dma_buf, 0, sizeof(img_rx_dma_buf));

    // 复用与遥控相同的 DMA 双缓冲初始化方式
    myUSART_DMAEx_MultiBuffer_Init(&huart1,(uint32_t *)img_rx_dma_buf[0],
    (uint32_t *)img_rx_dma_buf[1],VT_RX_BUF_SIZE);
}

/**
 * @brief 获取图传控制结构体的指针，仅用于只读访问。
 */
const imgtrans_ctrl_t *imgtrans_get_ctrl(void)
{
    return &s_imgtrans_ctrl;
}


void VT_data_is_error(void)
{
    // 使用最近一次收到的 21 字节帧做 CRC16 检验
    // 帧结构：前 VT_FRAME_LENGTH-2 字节为数据，最后 2 字节为 CRC16（小端）
    uint16_t recv_crc = (uint16_t)s_last_frame_buf[VT_FRAME_LENGTH - 2]
                       | (uint16_t)s_last_frame_buf[VT_FRAME_LENGTH - 1] << 8;

    uint16_t calc_crc = get_CRC16_check_sum(s_last_frame_buf,
                                            VT_FRAME_LENGTH - 2,
                                            0xFFFFu);   // 根据你 CRC 库的初值选择

    if (recv_crc != calc_crc)
    {
        // CRC 错误时，可在此清零通道或者打标志位，仿照 RC_data_is_error
        s_imgtrans_ctrl.rc.ch_0 = 0;
        s_imgtrans_ctrl.rc.ch_1 = 0;
        s_imgtrans_ctrl.rc.ch_2 = 0;
        s_imgtrans_ctrl.rc.ch_3 = 0;
    }
}
/**
 * @brief 图传发送函数：通过 USART1 发送一帧数据。
 *
 * @param data    待发送数据指针
 * @param length  待发送数据长度
 */
void imgtrans_send(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0)
    {
        return;
    }

    // 这里直接阻塞发送，如需异步/队列方式可自行扩展
    HAL_UART_Transmit(&huart1, (uint8_t *)data, length, HAL_MAX_DELAY);
}

/**
 * @brief USART1 + DMA 双缓冲接收事件处理函数。
 *
 *
 * @param huart UART 句柄
 * @param Size  本次 DMA 传输完成的长度（通常等于 IMGTRANS_RX_BUF_SIZE）
 */
void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart1)
    {
        return;
    }
    // 当前在用哪一块 Memory：CT=0 -> M0 在用，刚收完 M1；CT=1 -> M1 在用，刚收完 M0
    if (((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT) == RESET)
    {
        // 当前使用 Memory0，刚刚接收完成的是 Memory0，对应 img_rx_dma_buf[0]
        __HAL_DMA_DISABLE(huart->hdmarx);

        // 切换到 Memory1
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT_RX_BUF_SIZE);

        // 处理Memory0中的数据
        if(Size == VT_FRAME_LENGTH)
        {
            // 备份本帧原始数据用于 CRC 校验
            memcpy(s_last_frame_buf, img_rx_dma_buf[0], VT_FRAME_LENGTH);
            imgtrans_to_rc(img_rx_dma_buf[0], &s_imgtrans_ctrl);
        }
    }
    else
    {
        // 当前使用 Memory1，刚刚接收完成的是 Memory1，对应 img_rx_dma_buf[1]
        __HAL_DMA_DISABLE(huart->hdmarx);

        // 切换到 Memory0
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, VT_RX_BUF_SIZE);

        // 处理Memory1中的数据
        if(Size == VT_FRAME_LENGTH)
        {
            memcpy(s_last_frame_buf, img_rx_dma_buf[1], VT_FRAME_LENGTH);
            imgtrans_to_rc(img_rx_dma_buf[1], &s_imgtrans_ctrl);
        }
    }

    __HAL_DMA_ENABLE(huart->hdmarx);
}

/**
  * @brief          图传数据解析函数，将一帧图传数据解析为遥控数据
  * @param[in]      imgtrans_buf: 图传数据指针
  * @param[in]      imgtrans_ctrl: 图传控制结构体指针
  * @retval         none
  */
static void imgtrans_to_rc(volatile const uint8_t *imgtrans_buf, imgtrans_ctrl_t *imgtrans_ctrl)
{
    if (imgtrans_buf == NULL || imgtrans_ctrl == NULL)
    {
        return;
    }

    // 按照协议顺序解析：2 字节 SOF + 8 字节位域 + 7 字节鼠标 + 2 字节键盘 + 2 字节 CRC16 = 21 字节

    // 起始帧头
    imgtrans_ctrl->sof_1 = imgtrans_buf[0];
    imgtrans_ctrl->sof_2 = imgtrans_buf[1];

    // 解析 64bit 位域（小端：buf[2] 为最低字节）
    uint64_t packed = 0;
    packed |= (uint64_t)imgtrans_buf[2];
    packed |= (uint64_t)imgtrans_buf[3] << 8;
    packed |= (uint64_t)imgtrans_buf[4] << 16;
    packed |= (uint64_t)imgtrans_buf[5] << 24;
    packed |= (uint64_t)imgtrans_buf[6] << 32;
    packed |= (uint64_t)imgtrans_buf[7] << 40;
    packed |= (uint64_t)imgtrans_buf[8] << 48;
    packed |= (uint64_t)imgtrans_buf[9] << 56;

    // 依次取出各个 11/2/1bit 字段
    imgtrans_ctrl->rc.ch_0    =  (packed >> 0)  & 0x7FFu; // 11 bit
    imgtrans_ctrl->rc.ch_1    =  (packed >> 11) & 0x7FFu; // 11 bit
    imgtrans_ctrl->rc.ch_2    =  (packed >> 22) & 0x7FFu; // 11 bit
    imgtrans_ctrl->rc.ch_3    =  (packed >> 33) & 0x7FFu; // 11 bit
    imgtrans_ctrl->rc.mode_sw =  (packed >> 44) & 0x3u;   // 2 bit
    imgtrans_ctrl->rc.pause   =  (packed >> 46) & 0x1u;   // 1 bit
    imgtrans_ctrl->rc.fn_1    =  (packed >> 47) & 0x1u;   // 1 bit
    imgtrans_ctrl->rc.fn_2    =  (packed >> 48) & 0x1u;   // 1 bit
    imgtrans_ctrl->rc.wheel   =  (packed >> 49) & 0x7FFu; // 11 bit
    imgtrans_ctrl->rc.trigger =  (packed >> 60) & 0x1u;   // 1 bit

    // 鼠标和按键、CRC16 按字节解析（小端）
    const uint8_t *p = &imgtrans_buf[10];

    // mouse_x, mouse_y, mouse_z 为 int16，小端
    imgtrans_ctrl->mouse.mouse_x = (int16_t)( (int16_t)p[0] | (int16_t)p[1] << 8 );
    imgtrans_ctrl->mouse.mouse_y = (int16_t)( (int16_t)p[2] | (int16_t)p[3] << 8 );
    imgtrans_ctrl->mouse.mouse_z = (int16_t)( (int16_t)p[4] | (int16_t)p[5] << 8 );

    // 第 7 个字节携带三个 2bit 的按键状态：
    // bit1..0: 左键，bit3..2: 右键，bit5..4: 中键
    uint8_t mouse_btn = p[6];
    imgtrans_ctrl->mouse.mouse_left   =  mouse_btn        & 0x3u;
    imgtrans_ctrl->mouse.mouse_right  = (mouse_btn >> 2)  & 0x3u;
    imgtrans_ctrl->mouse.mouse_middle = (mouse_btn >> 4)  & 0x3u;

    // 键盘 2 字节，紧跟在鼠标 7 字节之后
    p = &imgtrans_buf[17];
    imgtrans_ctrl->key.key = (uint16_t)( (uint16_t)p[0] | (uint16_t)p[1] << 8 );

    // CRC16 在最后 2 字节
    p = &imgtrans_buf[19];
    imgtrans_ctrl->crc16 = (uint16_t)( (uint16_t)p[0] | (uint16_t)p[1] << 8 );

    imgtrans_ctrl->rc.ch_0 -= VT_CH_VALUE_OFFSET;
    imgtrans_ctrl->rc.ch_1 -= VT_CH_VALUE_OFFSET;
    imgtrans_ctrl->rc.ch_2 -= VT_CH_VALUE_OFFSET;
    imgtrans_ctrl->rc.ch_3 -= VT_CH_VALUE_OFFSET;
}
