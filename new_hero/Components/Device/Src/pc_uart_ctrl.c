/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : pc_uart_ctrl.c
 * @brief          : PC 上位机串口控制模块
 *                   通过 UART + DMA 双缓冲接收上位机键鼠控制数据，
 *                   帧格式与图传模块 H7 版本 21 字节协议完全兼容。
 * @author         :
 * @date           : 2026/03/07
 * @version        : v1.0
 ******************************************************************************
 * @attention
 *   裁判系统图传链路协议帧格式（共 21 字节，小端序，cmd_id = 0x0304）：
 *
 *   ┌──────┬─────────────┬──────┬──────┬──────────┬─────────────────────────────────────────────────────────┬──────────┐
 *   │ B0   │  B1~B2      │  B3  │  B4  │  B5~B6   │  B7~B18（12 字节 data）                                 │ B19~B20  │
 *   │ SOF  │ data_length │ seq  │ CRC8 │ cmd_id   │ mx(2) my(2) mz(2) btn_l(1) btn_r(1) key(2) rsv(2)      │ CRC16    │
 *   │ 0xA5 │ 0x0C 0x00   │ 变化 │ 校验 │ 04 03    │                                                         │ 整包校验 │
 *   └──────┴─────────────┴──────┴──────┴──────────┴─────────────────────────────────────────────────────────┴──────────┘
 *
 *   帧头 CRC8 覆盖字节 0~3（4 字节），整包 CRC16 覆盖字节 0~18（19 字节）。
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "pc_uart_ctrl.h"
#include "crc.h"

/* Private macros ------------------------------------------------------------*/

/** @brief 在线计数器重置值（每收到一帧后恢复此值，递减至 0 则判定为离线） */
#define PC_CTRL_ONLINE_CNT_RESET 0xFAU

/* Exported variables --------------------------------------------------------*/

/** @brief DMA 接收双缓冲区 */
uint8_t PC_Ctrl_MultiRx_Buf[2][PC_CTRL_RX_BUF_NUM];

/* Private variables ---------------------------------------------------------*/

/** @brief 当前绑定的 UART 句柄（由 pc_uart_ctrl_init 设置） */
static UART_HandleTypeDef *s_pc_ctrl_huart = NULL;

/** @brief PC 控制数据结构体，初始化为通信中断状态 */
static PC_Ctrl_Info_t s_pc_ctrl = {
    .rc_lost = true,
    .online_cnt = PC_CTRL_ONLINE_CNT_RESET,
};

/* Private function prototypes -----------------------------------------------*/

static void PC_UART_DMA_MultiBufferStart(UART_HandleTypeDef *huart,
                                         uint32_t *DstAddress,
                                         uint32_t *SecondMemAddress,
                                         uint32_t DataLength);

static void pc_frame_parse(const uint8_t *buf, PC_Ctrl_Info_t *ctrl);

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  初始化 PC 串口控制模块，配置 DMA 双缓冲接收。
 * @param  huart  指定 UART 句柄；传 NULL 时使用 PC_CTRL_DEFAULT_HUART。
 * @retval None
 */
void pc_uart_ctrl_init(UART_HandleTypeDef *huart)
{
  s_pc_ctrl_huart = (huart != NULL) ? huart : PC_CTRL_DEFAULT_HUART;

  memset(&s_pc_ctrl, 0, sizeof(s_pc_ctrl));
  memset(PC_Ctrl_MultiRx_Buf, 0, sizeof(PC_Ctrl_MultiRx_Buf));

  s_pc_ctrl.rc_lost = true;
  s_pc_ctrl.online_cnt = PC_CTRL_ONLINE_CNT_RESET;

  PC_UART_DMA_MultiBufferStart(s_pc_ctrl_huart,
                               (uint32_t *)PC_Ctrl_MultiRx_Buf[0],
                               (uint32_t *)PC_Ctrl_MultiRx_Buf[1],
                               PC_CTRL_RX_BUF_NUM);
}
//------------------------------------------------------------------------------

/**
 * @brief  PC 串口 DMA 接收事件处理函数（IDLE 中断触发）。
 * @param  huart  触发事件的 UART 句柄
 * @param  Size   本次 DMA 实际接收字节数
 * @retval None
 * @note   DMA 的最终重新使能由 bsp_uart.c 中 HAL_UARTEx_RxEventCallback
 *         末尾统一执行，本函数只负责切换缓冲区和解析数据。
 */
void PC_Ctrl_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* 仅处理已注册的串口，不匹配则立即返回 */
  if (huart != s_pc_ctrl_huart)
  {
    return;
  }

  /* 判断 DMA 当前正在写入哪块缓冲区（CT=0 表示 Memory0 在用，刚完成的是 Memory0） */
  if ((((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR & DMA_SxCR_CT) == RESET)
  {
    /* 禁用 DMA，切换至 Memory1，重置传输计数 */
    __HAL_DMA_DISABLE(huart->hdmarx);
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
    __HAL_DMA_SET_COUNTER(huart->hdmarx, PC_CTRL_RX_BUF_NUM);

    /* 解析 Memory0 中刚接收完毕的帧 */
    if (Size == PC_CTRL_FRAME_LENGTH)
    {
      pc_frame_parse(PC_Ctrl_MultiRx_Buf[0], &s_pc_ctrl);
    }
  }
  else
  {
    /* 禁用 DMA，切换至 Memory0，重置传输计数 */
    __HAL_DMA_DISABLE(huart->hdmarx);
    ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR &= ~DMA_SxCR_CT;
    __HAL_DMA_SET_COUNTER(huart->hdmarx, PC_CTRL_RX_BUF_NUM);

    /* 解析 Memory1 中刚接收完毕的帧 */
    if (Size == PC_CTRL_FRAME_LENGTH)
    {
      pc_frame_parse(PC_Ctrl_MultiRx_Buf[1], &s_pc_ctrl);
    }
  }
}
//------------------------------------------------------------------------------

/**
 * @brief  获取 PC 串口控制数据的只读指针。
 * @retval 指向内部 PC_Ctrl_Info_t 结构体的常量指针
 */
const PC_Ctrl_Info_t *get_pc_uart_ctrl_point(void)
{
  return &s_pc_ctrl;
}
//------------------------------------------------------------------------------

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  配置 UART 的 DMA 双缓冲循环接收（风格参照 bsp_uart.c）。
 * @param  huart             UART 句柄
 * @param  DstAddress        DMA Memory0 缓冲区地址
 * @param  SecondMemAddress  DMA Memory1 缓冲区地址
 * @param  DataLength        单块缓冲区字节数
 * @retval None
 */
static void PC_UART_DMA_MultiBufferStart(UART_HandleTypeDef *huart,
                                         uint32_t *DstAddress,
                                         uint32_t *SecondMemAddress,
                                         uint32_t DataLength)
{
  /* 设置接收模式为 IDLE 触发 */
  huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

  /* 记录单次 DMA 传输长度（供 HAL 内部使用） */
  huart->RxXferSize = (uint16_t)DataLength;

  /* 使能 UART DMA 接收请求 */
  SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

  /* 使能 IDLE 中断 */
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

  /* 等待 DMA 流完全停止 */
  do
  {
    __HAL_DMA_DISABLE(huart->hdmarx);
  } while (huart->hdmarx->Instance->CR & DMA_SxCR_EN);

  /* 配置外设地址（UART 数据寄存器） */
  huart->hdmarx->Instance->PAR = (uint32_t)&huart->Instance->DR;

  /* 配置双缓冲区地址 */
  huart->hdmarx->Instance->M0AR = (uint32_t)DstAddress;
  huart->hdmarx->Instance->M1AR = (uint32_t)SecondMemAddress;

  /* 设置传输数据量 */
  huart->hdmarx->Instance->NDTR = DataLength;

  /* 使能双缓冲模式 */
  SET_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_DBM);

  /* 启动 DMA */
  __HAL_DMA_ENABLE(huart->hdmarx);
}
//------------------------------------------------------------------------------

/**
 * @brief  解析 H7 图传自定义 21 字节帧，更新控制结构体。
 *
 *   帧结构：
 *     [0]     sof_1 = 0xA9
 *     [1]     sof_2 = 0x53
 *     [2:9]   RC 数据（64bit 小端位域）：
 *               ch_0:11  ch_1:11  ch_2:11  ch_3:11
 *               mode_sw:2  pause:1  fn_1:1  fn_2:1
 *               wheel:11  trigger:1
 *     [10:16] 鼠标：mouse_x(2) mouse_y(2) mouse_z(2) btn(1)
 *               btn bit[1:0]=左键  bit[3:2]=右键  bit[5:4]=中键
 *     [17:18] 键盘 uint16（与 remote_ctrl.key.v 位图定义一致）
 *     [19:20] CRC16（小端，覆盖字节 0~18，初始值 0xFFFF）
 *
 * @param  buf   原始接收缓冲区指针（PC_CTRL_FRAME_LENGTH 字节）
 * @param  ctrl  目标控制结构体指针
 * @retval None
 */
static void pc_frame_parse(const uint8_t *buf, PC_Ctrl_Info_t *ctrl)
{
  if (buf == NULL || ctrl == NULL)
  {
    return;
  }

  /* ---- 帧头双字节校验 ---- */
  if (buf[0] != PC_CTRL_SOF1 || buf[1] != PC_CTRL_SOF2)
  {
    return;
  }

  /* ---- CRC16 校验（覆盖字节 0~18，初始值 0xFFFF）---- */
  uint16_t recv_crc = (uint16_t)buf[PC_CTRL_FRAME_LENGTH - 2] | (uint16_t)buf[PC_CTRL_FRAME_LENGTH - 1] << 8;
  uint16_t calc_crc = get_CRC16_check_sum((uint8_t *)buf,
                                          PC_CTRL_FRAME_LENGTH - 2,
                                          0xFFFFu);
  if (recv_crc != calc_crc)
  {
    return; /* CRC 校验失败，丢弃本帧 */
  }

  /* ---- 解析 RC 64bit 位域（字节 2~9，小端序）---- */
  uint64_t packed = 0;
  packed |= (uint64_t)buf[2];
  packed |= (uint64_t)buf[3] << 8;
  packed |= (uint64_t)buf[4] << 16;
  packed |= (uint64_t)buf[5] << 24;
  packed |= (uint64_t)buf[6] << 32;
  packed |= (uint64_t)buf[7] << 40;
  packed |= (uint64_t)buf[8] << 48;
  packed |= (uint64_t)buf[9] << 56;

  /* 摇杆通道（各 11bit），减去中点偏移量 */
  ctrl->rc.ch[0] = (int16_t)((packed >> 0) & 0x7FFu) - PC_CTRL_CH_VALUE_OFFSET;
  ctrl->rc.ch[1] = (int16_t)((packed >> 11) & 0x7FFu) - PC_CTRL_CH_VALUE_OFFSET;
  ctrl->rc.ch[2] = (int16_t)((packed >> 22) & 0x7FFu) - PC_CTRL_CH_VALUE_OFFSET;
  ctrl->rc.ch[3] = (int16_t)((packed >> 33) & 0x7FFu) - PC_CTRL_CH_VALUE_OFFSET;
  ctrl->rc.mode_sw = (uint8_t)((packed >> 44) & 0x3u);
  ctrl->rc.pause = (uint8_t)((packed >> 46) & 0x1u);
  ctrl->rc.fn_1 = (uint8_t)((packed >> 47) & 0x1u);
  ctrl->rc.fn_2 = (uint8_t)((packed >> 48) & 0x1u);
  ctrl->rc.wheel = (int16_t)((packed >> 49) & 0x7FFu) - PC_CTRL_CH_VALUE_OFFSET;
  ctrl->rc.trigger = (uint8_t)((packed >> 60) & 0x1u);

  /* ---- 解析鼠标数据（字节 10~16）---- */
  const uint8_t *p = &buf[10];
  ctrl->mouse.x = (int16_t)((int16_t)p[0] | (int16_t)p[1] << 8); /* mouse_x */
  ctrl->mouse.y = (int16_t)((int16_t)p[2] | (int16_t)p[3] << 8); /* mouse_y */
  ctrl->mouse.z = (int16_t)((int16_t)p[4] | (int16_t)p[5] << 8); /* mouse_z */
  ctrl->mouse.press_l = (p[6] >> 0) & 0x3u;                      /* 左键状态 */
  ctrl->mouse.press_r = (p[6] >> 2) & 0x3u;                      /* 右键状态 */
  ctrl->mouse.press_m = (p[6] >> 4) & 0x3u;                      /* 中键状态 */

  /* ---- 解析键盘数据（字节 17~18）---- */
  p = &buf[17];
  ctrl->key.v = (uint16_t)((uint16_t)p[0] | (uint16_t)p[1] << 8);

  /* ---- 更新在线状态 ---- */
  ctrl->online_cnt = PC_CTRL_ONLINE_CNT_RESET;
  ctrl->rc_lost = false;
}
