/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       imgtrans_control.c/h
  * @brief      图传（图像传输）控制模块，基于 USART10 收发。
  *             本模块只做串口数据搬运，不解析具体协议：
  *              - 上位机 → 板端：通过中断或 DMA 收到的数据写入接收缓冲区；
  *              - 板端 → 上位机：通过发送接口将待发送图像/控制数据从缓冲区发出。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef IMGTRANS_CONTROL_H
#define IMGTRANS_CONTROL_H

#include "struct_typedef.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VT_RX_BUF_SIZE 21u
#define VT_FRAME_LENGTH 21u
#define VT_TX_BUF_SIZE 256u

#define VT_CH_VALUE_MIN         ((uint16_t)364)
#define VT_CH_VALUE_OFFSET      ((uint16_t)1024)
#define VT_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define VT_SW_C                ((uint16_t)0)
#define VT_SW_N                ((uint16_t)1)
#define VT_SW_S                ((uint16_t)2)

/* ----------------------- Data Struct ------------------------------------- */
/**
 * @brief  图传控制内部状态结构体。
 */
typedef struct
{
    uint8_t sof_1;
    uint8_t sof_2;
    PACKED struct
    {
        int16_t ch_0:11;
        int16_t ch_1:11;
        int16_t ch_2:11;
        int16_t ch_3:11;
        uint8_t mode_sw:2;
        uint8_t pause:1;
        uint8_t fn_1:1;
        uint8_t fn_2:1;
        int16_t wheel:11;
        uint8_t trigger:1;
    } rc;
    PACKED struct
    {
        int64_t mouse_x;
        int64_t mouse_y;
        int64_t mouse_z;
        uint8_t mouse_left:2;
        uint8_t mouse_right:2;
        uint8_t mouse_middle:2;
    } mouse;
    PACKED struct
    {
        uint16_t key;
    } key;
    uint16_t crc16;

    PACKED uint8_t  tx_buf[VT_TX_BUF_SIZE];         /**< 发送缓冲区 */
    PACKED uint16_t tx_len;                        /**< 本帧待发送长度 */
} imgtrans_ctrl_t, *imgtrans_ctrl_t_ptr;

/**
 * @brief  图传模块初始化（绑定 USART10，并启动接收）。
 */
void imgtrans_init(void);

/**
 * @brief  获取图传控制结构体常量指针，仅用于读取状态。
 */
const imgtrans_ctrl_t *imgtrans_get_ctrl(void);

/**
 * @brief  图传发送接口，将 data 中的 length 字节通过 USART10 发出。
 * @note   内部为阻塞式发送（HAL_UART_Transmit）。如需 DMA/中断方式，可自行扩展实现。
 */
void imgtrans_send(const uint8_t *data, uint16_t length);

/**
 * @brief  USART10 接收完成回调（在 HAL_UART_RxCpltCallback 中调用）。
 * @param  huart  UART 句柄指针
 */
void USER_USART1_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif // IMGTRANS_CONTROL_H
