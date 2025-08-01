#include "Ros_Task.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
//
// // 队列定义
// osMessageQId ros_data_queueHandle;
//
// // UART6 DMA缓冲区
// #define UART6_DMA_RX_BUF_SIZE 256
// static uint8_t uart6_dma_rx_buf[UART6_DMA_RX_BUF_SIZE];
// static volatile uint16_t uart6_dma_last_pos = 0;
//
// // 解析缓冲区
// static uint8_t parser_buf[ROS_MAX_DATA_LEN + 8];
// static uint16_t parser_buf_len = 0;

void Ros_Task(void const * argument)
{
//     // 创建数据队列
//     osMessageQDef(ros_data_queue, 8, RosParsedData_t);
//     ros_data_queueHandle = osMessageCreate(osMessageQ(ros_data_queue), NULL);
//
//     // 启动UART6循环DMA接收
//     HAL_UART_Receive_DMA(&huart6, uart6_dma_rx_buf, UART6_DMA_RX_BUF_SIZE);

    // ParserState_t state = STATE_WAIT_HEADER;
    // uint8_t id = 0, len = 0, checksum = 0, calc_sum = 0;
    // uint8_t data_buf[ROS_MAX_DATA_LEN];
    // uint8_t data_idx = 0;

    for(;;)
    {
        // // 计算DMA当前写入位置
        // uint16_t cur_pos = UART6_DMA_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart6.hdmarx);
        // while (uart6_dma_last_pos != cur_pos) {
        //     uint8_t byte = uart6_dma_rx_buf[uart6_dma_last_pos];
        //     uart6_dma_last_pos = (uart6_dma_last_pos + 1) % UART6_DMA_RX_BUF_SIZE;
        //
        //     switch (state) {
        //         case STATE_WAIT_HEADER:
        //             if (byte == ROS_PACKET_HEADER) {
        //                 calc_sum = byte;
        //                 state = STATE_WAIT_ID;
        //             }
        //             break;
        //         case STATE_WAIT_ID:
        //             id = byte;
        //             calc_sum += byte;
        //             state = STATE_WAIT_LEN;
        //             break;
        //         case STATE_WAIT_LEN:
        //             len = byte;
        //             calc_sum += byte;
        //             if (len > 0 && len <= ROS_MAX_DATA_LEN) {
        //                 data_idx = 0;
        //                 state = STATE_WAIT_DATA;
        //             } else {
        //                 state = STATE_WAIT_HEADER;
        //             }
        //             break;
        //         case STATE_WAIT_DATA:
        //             data_buf[data_idx++] = byte;
        //             calc_sum += byte;
        //             if (data_idx >= len) {
        //                 state = STATE_WAIT_CHECKSUM;
        //             }
        //             break;
        //         case STATE_WAIT_CHECKSUM:
        //             checksum = byte;
        //             if ((calc_sum & 0xFF) == checksum && len >= 12) {
        //                 // 解析数据区
        //                 RosParsedData_t parsed;
        //                 memcpy(&parsed.yaw,   &data_buf[0], 4);
        //                 memcpy(&parsed.pitch, &data_buf[4], 4);
        //                 memcpy(&parsed.state, &data_buf[8], 4);
        //                 osMessagePut(ros_data_queueHandle, *(uint32_t*)&parsed, 0);
        //             }
        //             state = STATE_WAIT_HEADER;
        //             break;
        //         default:
        //             state = STATE_WAIT_HEADER;
        //             break;
        //     }
        // }
        osDelay(1);
    }
}