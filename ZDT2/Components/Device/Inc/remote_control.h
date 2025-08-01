/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		36u

/**
 * @brief Length of remote control received data
 */
#define RC_FRAME_LENGTH		18u

/**
 * @brief 上位机通信协议相关定义
 */
#define PC_PACKET_HEADER        0x42
#define PC_RX_BUF_SIZE         64u
#define PC_MAX_DATA_LEN        32u
#define PC_MIN_PACKET_SIZE     4u   // 包头+ID+长度+校验和

/**
 * @brief 上位机发送协议相关定义
 */
#define PC_TX_PACKET_HEADER     0x42    // 与接收协议保持一致
#define PC_TX_BUF_SIZE         32u
#define PC_TX_DATA_LEN         8u       // 两个浮点数角度值
#define PC_TX_PACKET_ID        0x01     // 固定ID为0x01

/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/**
 * @brief judgement keyboard set short time
 */
#define KEY_SET_SHORT_TIME		50U
/**
 * @brief judgement keyboard set long time
 */
#define KEY_SET_LONG_TIME		1000U

/**
 * @brief status of keyboard up
 */
#define KEY_UP                    0x00U
/**
 * @brief status of keyboard down
 */
#define KEY_DOWN                  0x01U

/**
 * @brief MAX speed of mouse speed
 */
#define MOUSE_SPEED_MAX		300U

/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef enum that contains the status of the keyboard.
 */
typedef enum
{
	UP,			/*!< up */
	SHORT_DOWN,	/*!< short time down */
	DOWN,		/*!< long time down */
	PRESS,		/*!< 0->1 */
	RELAX,		/*!< 1->0 */
	KeyBoard_Status_NUM,
}KeyBoard_Status_e;

typedef struct
{
	uint16_t Count;
	KeyBoard_Status_e Status;
	KeyBoard_Status_e last_Status;
	bool last_KEY_PRESS;
	bool KEY_PRESS;
}KeyBoard_Info_Typedef;

typedef struct
{
	KeyBoard_Info_Typedef press_l;
	KeyBoard_Info_Typedef press_r;
	KeyBoard_Info_Typedef W;
	KeyBoard_Info_Typedef S;
	KeyBoard_Info_Typedef A;
	KeyBoard_Info_Typedef D;
	KeyBoard_Info_Typedef SHIFT;
	KeyBoard_Info_Typedef CTRL;
	KeyBoard_Info_Typedef Q;
	KeyBoard_Info_Typedef E;
	KeyBoard_Info_Typedef R;
	KeyBoard_Info_Typedef F;
	KeyBoard_Info_Typedef G;
	KeyBoard_Info_Typedef Z;
	KeyBoard_Info_Typedef X;
	KeyBoard_Info_Typedef C;
	KeyBoard_Info_Typedef V;
	KeyBoard_Info_Typedef B;
}Remote_Pressed_Typedef;

/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
	/**
	 * @brief structure that contains the information for the lever/Switch.
	 */
	struct
	{
		int16_t ch[5];
		uint8_t s[2];
	} rc;
	
	/**
	 * @brief structure that contains the information for the mouse.
	 */
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	/**
	 * @brief structure that contains the information for the keyboard.
	 */
	union
	{
		uint16_t v;
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		} set;
	} key;

	bool rc_lost;   /*!< lost flag */
	uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/**
 * @brief 上位机数据包结构体
 */
typedef struct
{
    float yaw_angle;      // YAW轴目标角度
    float pitch_angle;    // PITCH轴目标角度
    bool data_valid;      // 数据有效标志
    uint8_t packet_id;    // 数据包ID
    uint32_t last_update; // 最后更新时间戳
} PC_Data_Typedef;

/**
 * @brief 上位机发送数据包结构体
 */
typedef struct
{
    float yaw_angle;      // YAW轴实时角度
    float pitch_angle;    // PITCH轴实时角度
    uint8_t packet_id;    // 数据包ID
} PC_TX_Data_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief 上位机通信相关变量
 */
extern PC_Data_Typedef pc_data;
extern uint8_t PC_MultiRx_Buf[2][PC_RX_BUF_SIZE];

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* Mouse Exported defines -----------------------------------------------------*/
#define MOUSE_X_MOVE_SPEED    (remote_ctrl.mouse.x )
#define MOUSE_Y_MOVE_SPEED    (remote_ctrl.mouse.y )
#define MOUSE_Z_MOVE_SPEED    (remote_ctrl.mouse.z )
#define MOUSE_PRESSED_LEFT    (remote_ctrl.mouse.press_l)
#define MOUSE_PRESSED_RIGHT   (remote_ctrl.mouse.press_r)

/* KeyBoard Exported defines --------------------------------------------------*/
#define KeyBoard_W            (remote_ctrl.key.set.W)
#define KeyBoard_S            (remote_ctrl.key.set.S)
#define KeyBoard_A            (remote_ctrl.key.set.A)
#define KeyBoard_D            (remote_ctrl.key.set.D)
#define KeyBoard_SHIFT        (remote_ctrl.key.set.SHIFT)
#define KeyBoard_CTRL         (remote_ctrl.key.set.CTRL)
#define KeyBoard_Q            (remote_ctrl.key.set.Q)
#define KeyBoard_E            (remote_ctrl.key.set.E)
#define KeyBoard_R            (remote_ctrl.key.set.R)
#define KeyBoard_F            (remote_ctrl.key.set.F)
#define KeyBoard_G            (remote_ctrl.key.set.G)
#define KeyBoard_Z            (remote_ctrl.key.set.Z)
#define KeyBoard_X            (remote_ctrl.key.set.X)
#define KeyBoard_C            (remote_ctrl.key.set.C)
#define KeyBoard_V            (remote_ctrl.key.set.V)
#define KeyBoard_B            (remote_ctrl.key.set.B)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)
/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);

/**
  * @brief  解析上位机数据包
  */
extern void PC_Packet_Parse(volatile const uint8_t *pc_buf, uint16_t size, PC_Data_Typedef *pc_data);

/**
  * @brief  发送电机实时角度值给上位机
  */
extern void PC_Send_Motor_Angles(float yaw_angle, float pitch_angle, uint8_t packet_id);

/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  Update the Remote Control Information according the CAN2
  */
extern void Remote_Info_Update(uint32_t *StdId, uint8_t *rxBuf,Remote_Info_Typedef *remote_ctrl);

extern bool Key_W(void);
extern bool Key_S(void);
extern bool Key_Shift(void);
extern bool Key_Ctrl(void);
extern bool Key_Q(void);
extern bool Key_F(void);
extern bool Key_C(void);
extern bool Key_Z(void);

extern Remote_Info_Typedef remote_ctrl;
const Remote_Info_Typedef *get_remote_control_point();
#endif //REMOTE_CONTROL_H
