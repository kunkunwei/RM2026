/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : pc_uart_ctrl.h
 * @brief          : PC 上位机串口控制模块接口定义
 *                   通过 UART + DMA 双缓冲接收图传链路键鼠控制数据。
 *                   帧格式遵循裁判系统官方协议，cmd_id = 0x0304，共 21 字节。
 * @author         :
 * @date           : 2026/03/07
 * @version        : v1.0
 ******************************************************************************
 * @attention
 *   【串口切换方法】
 *   仅需修改下方 PC_CTRL_DEFAULT_HUART 宏为目标串口句柄，例如：
 *     &huart2  ->  USART2
 *     &huart6  ->  USART6（默认）
 *   并在 bsp_uart.c 的 HAL_UARTEx_RxEventCallback 中，
 *   为新串口添加对应的 else-if 分支调用 PC_Ctrl_RxHandler。
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PC_UART_CTRL_H
#define PC_UART_CTRL_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "usart.h" /* 提供 huart1 / huart3 / huart6 等句柄声明 */

/* Exported defines ----------------------------------------------------------*/

/** @brief 接收 DMA 双缓冲区单块字节数（须 ≥ PC_CTRL_FRAME_LENGTH） */
#define PC_CTRL_RX_BUF_NUM 36u

/** @brief 单帧总字节数（21 字节，与图传模块 H7 自定义协议一致）*/
#define PC_CTRL_FRAME_LENGTH     21u

/** @brief 帧头第一字节（上位机发送的固定帧头） */
#define PC_CTRL_SOF1             0xA9u

/** @brief 帧头第二字节（上位机发送的固定帧头） */
#define PC_CTRL_SOF2             0x53u

/** @brief 摇杆通道原始值中点偏移量（与 SBUS 协议一致） */
#define PC_CTRL_CH_VALUE_OFFSET  ((int16_t)1024)

/**
 * @brief  默认绑定的 UART 句柄。
 * @note   修改此宏即可一键切换串口，同时在 bsp_uart.c 回调中
 *         为新串口添加对应 else-if 分支：
 *           &huart1  ->  USART1
 *           &huart3  ->  USART3
 *           &huart6  ->  USART6（默认）
 */
#define PC_CTRL_DEFAULT_HUART (&huart6)

/* RC Switch Definition -------------------------------------------------------*/
#define PC_CTRL_SW_C ((uint16_t)0) /*!< 拨杆下位 */
#define PC_CTRL_SW_N ((uint16_t)1) /*!< 拨杆中位 */
#define PC_CTRL_SW_S ((uint16_t)2) /*!< 拨杆上位 */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief  PC 串口控制信息结构体。
 * @note   字段布局与 Remote_Info_Typedef 对齐，方便上层代码无缝切换数据来源。
 */
typedef struct
{
  /**
   * @brief 摇杆 / 拨杆 / 功能键数据
   */
  struct
  {
    int16_t ch[4];   /*!< 摇杆通道 0~3，已减去偏移量，有效范围约 [-660, 660] */
    int16_t wheel;   /*!< 滚轮通道，已减去偏移量 */
    uint8_t mode_sw; /*!< 模式拨杆 (0/1/2) */
    uint8_t pause;   /*!< 暂停标志位 */
    uint8_t fn_1;    /*!< 功能键 1 */
    uint8_t fn_2;    /*!< 功能键 2 */
    uint8_t trigger; /*!< 扳机键 */
  } rc;

  /**
   * @brief 鼠标数据
   */
  struct
  {
    int16_t x;       /*!< X 轴速度 */
    int16_t y;       /*!< Y 轴速度 */
    int16_t z;       /*!< Z 轴速度 */
    uint8_t press_l; /*!< 左键状态 */
    uint8_t press_r; /*!< 右键状态 */
    uint8_t press_m; /*!< 中键状态 */
  } mouse;

  /**
   * @brief 键盘数据，键位定义与 Remote_Info_Typedef 完全一致
   */
  union
  {
    uint16_t v;
    struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } set;
  } key;

  bool rc_lost;       /*!< 数据丢失标志（true = 通信中断） */
  uint8_t online_cnt; /*!< 在线计数器，每帧接收后重置，可用于超时检测 */

} PC_Ctrl_Info_t;

/* KeyBoard Exported defines --------------------------------------------------*/
#define PC_KeyBoard_W (get_pc_uart_ctrl_point()->key.set.W)
#define PC_KeyBoard_S (get_pc_uart_ctrl_point()->key.set.S)
#define PC_KeyBoard_A (get_pc_uart_ctrl_point()->key.set.A)
#define PC_KeyBoard_D (get_pc_uart_ctrl_point()->key.set.D)
#define PC_KeyBoard_SHIFT (get_pc_uart_ctrl_point()->key.set.SHIFT)
#define PC_KeyBoard_CTRL (get_pc_uart_ctrl_point()->key.set.CTRL)
#define PC_KeyBoard_Q (get_pc_uart_ctrl_point()->key.set.Q)
#define PC_KeyBoard_E (get_pc_uart_ctrl_point()->key.set.E)
#define PC_KeyBoard_R (get_pc_uart_ctrl_point()->key.set.R)
#define PC_KeyBoard_F (get_pc_uart_ctrl_point()->key.set.F)
#define PC_KeyBoard_G (get_pc_uart_ctrl_point()->key.set.G)
#define PC_KeyBoard_Z (get_pc_uart_ctrl_point()->key.set.Z)
#define PC_KeyBoard_X (get_pc_uart_ctrl_point()->key.set.X)
#define PC_KeyBoard_C (get_pc_uart_ctrl_point()->key.set.C)
#define PC_KeyBoard_V (get_pc_uart_ctrl_point()->key.set.V)
#define PC_KeyBoard_B (get_pc_uart_ctrl_point()->key.set.B)

/* Exported variables ---------------------------------------------------------*/

/**
 * @brief DMA 接收双缓冲区（在 bsp_uart.c 回调中间接引用）
 */
extern uint8_t PC_Ctrl_MultiRx_Buf[2][PC_CTRL_RX_BUF_NUM];

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  初始化 PC 串口控制模块，配置指定串口的 DMA 双缓冲接收。
 * @param  huart  UART 句柄指针；传 NULL 时使用 PC_CTRL_DEFAULT_HUART。
 * @note   在 BSP_USART_Init() 末尾调用。
 */
extern void pc_uart_ctrl_init(UART_HandleTypeDef *huart);

/**
 * @brief  PC 串口 DMA 接收事件处理函数（IDLE 中断触发）。
 * @param  huart  触发事件的 UART 句柄
 * @param  Size   本次 DMA 实际接收字节数
 * @note   在 HAL_UARTEx_RxEventCallback 中对应串口分支内调用。
 *         函数内部会校验 huart 是否匹配已注册句柄，不匹配则立即返回。
 */
extern void PC_Ctrl_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

/**
 * @brief  获取 PC 串口控制数据的只读指针。
 * @retval 指向内部 PC_Ctrl_Info_t 结构体的常量指针
 */
extern const PC_Ctrl_Info_t *get_pc_uart_ctrl_point(void);

#endif /* PC_UART_CTRL_H */
