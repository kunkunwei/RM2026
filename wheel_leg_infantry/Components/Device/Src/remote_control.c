/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : remote_control.c
  * @brief          : remote_control interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/05/23
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "remote_control.h"
#include "ramp.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
	.rc_lost = true,
	.online_cnt = 0xFAU,
};
//云台CAN帧控制底盘遥控器信息结构体变量
Chassis_RC_Info_t chassis_can_rc_info;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/* Private variables ---------------------------------------------------------*/
/**
 * @brief structure that contains the information of keyboard
 */
static Remote_Pressed_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Update the status of keyboard
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */
		remote_ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		remote_ctrl->rc_lost = false;
}
//------------------------------------------------------------------------------

/**
  * @brief  Update the Remote Control Information according the CAN2
  * @param  StdId  pointer to the specifies the standard identifier.
  * @param  rxBuf  pointer to the can receive data
  * @param  remote_ctrl pointer to a Remote_Info_Typedef structure 
  *         that contains the information of Remote Control
  * @retval None
  */
void Remote_Info_Update(uint32_t *StdId, uint8_t *rxBuf,Remote_Info_Typedef *remote_ctrl)
{
	if(*StdId != 0x302)
	{
		return;
	}
	
	remote_ctrl->rc.s[0]  = (uint8_t)(rxBuf[0] & 0xC0U) >> 6;
	remote_ctrl->rc.ch[4] = (uint8_t)(rxBuf[0] & 0x20U) >> 5;
	if(remote_ctrl->rc.ch[4] == 0)
	{
		remote_ctrl->rc.ch[4] = -1*((uint8_t)(rxBuf[0] & 0x10U) >> 4);
	}
	
	// UI_Char.Fire = (uint8_t)(rxBuf[1] & 0x10U) >> 4;
	// UI_Char.Shooter_Mode = (uint8_t)(rxBuf[1] & 0x06U) >> 2;
	// UI_Char.Auto  = (uint8_t)(rxBuf[1] & 0x02U) >> 1;
	// UI_Char.Cover = (uint8_t)(rxBuf[1] & 0x01U);
	
	remote_ctrl->rc.ch[3] = (int16_t)rxBuf[2] << 8 | rxBuf[3];
	remote_ctrl->rc.ch[2] = (int16_t)rxBuf[4] << 8 | rxBuf[5];
	remote_ctrl->key.v    = (int16_t)rxBuf[6] << 8 | rxBuf[7];
}

const Remote_Info_Typedef *get_remote_control_point(){
  return &remote_ctrl;
}
/**
 * @brief  解析底盘控制帧数据
 * @details 对应 Chassis_Ctrl_Can_Send 发送的数据格式 (压缩编码)
 * @param  rc_info: 指向存储解析后数据的结构体指针
 * @param  data: CAN接收到的8字节数据数组
 */
void get_chassis_ctrl_measure(Chassis_RC_Info_t *rc_info, uint8_t *data)
{
	// 解压 12-bit 模拟通道数据
	// Byte 0: X[11:4]
	// Byte 1: X[3:0] | WZ[11:8]
	// Byte 2: WZ[7:0]
	// Byte 3: R1[11:4]
	// Byte 4: R1[3:0] | R2[11:8]
	// Byte 5: R2[7:0]

	int16_t x_raw  = ((int16_t)data[0] << 4) | (data[1] >> 4);
	int16_t wz_raw = ((int16_t)(data[1] & 0x0F) << 8) | data[2];
	int16_t r1_raw = ((int16_t)data[3] << 4) | (data[4] >> 4);
	int16_t r2_raw = ((int16_t)(data[4] & 0x0F) << 8) | data[5];

	// 12-bit 符号扩展 (如果最高位 bit11 为 1，则扩展符号位)
	if (x_raw & 0x0800) x_raw |= 0xF000;
	if (wz_raw & 0x0800) wz_raw |= 0xF000;
	if (r1_raw & 0x0800) r1_raw |= 0xF000;
	if (r2_raw & 0x0800) r2_raw |= 0xF000;

	rc_info->ch[0] = x_raw;
	rc_info->ch[1] = wz_raw;
	rc_info->ch[2] = r1_raw;
	rc_info->ch[3] = r2_raw;

	// rc_info->x = x_raw;
	// rc_info->w_z = wz_raw;
	// rc_info->roll_1 = r1_raw;
	// rc_info->roll_2 = r2_raw;

	// 解压开关数据 (Byte 6)
	// 映射: 0 -> -1, 1 -> 0, 2 -> 1
	// s[0] (L1): bits 1:0
	// s[1] (L2): bits 3:2
	// s[2] (R2): bits 5:4
	// s[3] (R1): bits 7:6

	uint8_t sw_byte = data[6];

	rc_info->s[0]  = (int8_t)(sw_byte & 0x03) - 1;
	rc_info->s[1]  = (int8_t)((sw_byte >> 2) & 0x03) - 1;
	rc_info->s[2] = (int8_t)((sw_byte >> 4) & 0x03) - 1;
	rc_info->s[3] = (int8_t)((sw_byte >> 6) & 0x03) - 1;
	// rc_info->left_switch_1  = (int8_t)(sw_byte & 0x03) - 1;
	// rc_info->left_switch_2  = (int8_t)((sw_byte >> 2) & 0x03) - 1;
	// rc_info->right_switch_2 = (int8_t)((sw_byte >> 4) & 0x03) - 1;
	// rc_info->right_switch_1 = (int8_t)((sw_byte >> 6) & 0x03) - 1;
}

const Chassis_RC_Info_t* get_Chassis_CAN_RC_Info()
{
	return &chassis_can_rc_info;
}
//------------------------------------------------------------------------------

// /**
//   * @brief  clear the remote control data while the device offline
//   * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
//   *         contains the information  for the remote control.
//   * @retval none
//   */
// void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
// {
//   /* Juege the device status */
//   if(remote_ctrl->online_cnt <= 0x32U)
//   {
//     /* clear the data */
//     memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

//     /* reset the online count */
//     remote_ctrl->online_cnt = 0xFAU;
		
//     /* set the lost flag */
// 		remote_ctrl->rc_lost = true;
//   }
//   else if(remote_ctrl->online_cnt > 0)
//   {
//     /* online count decrements which reseted in received interrupt  */
//     remote_ctrl->online_cnt--;
//   }
// }
// //------------------------------------------------------------------------------

// /**
//   * @brief  Update the status of keyboard
//   * @param  KeyInfo: pointer to a KeyBoard_Info_Typedef structure that
//   *         contains the information for the keyboard.
//   * @param  KeyBoard_Status: flag of the keyboard status 
//   * @retval none
//   */
// static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status)
// { 
//   /* store the keyboard status */
//   KeyInfo->KEY_PRESS = KeyBoard_Status;

//   /* judge the change of keyboard status */
//   if(KeyInfo->last_KEY_PRESS != KeyInfo->KEY_PRESS)
//   {
//     /* clear the status judgement count  */
//     KeyInfo->Count = 0;
  
//     /* update the last keyboard status */
//     KeyInfo->last_KEY_PRESS = KeyInfo->KEY_PRESS;
//   }

//   /* the keyboard status is key up */
//   if(KEY_UP == KeyInfo->KEY_PRESS)
//   {
//     /* last keyboard status is key down */
//     if(KEY_UP != KeyInfo->last_Status)
//       KeyInfo->Count++;
//     /* last keyboard status is key up */
//     else
//       KeyInfo->Count = 0;

//     /* update the keyboard status according the judgement count */
//     if(KeyInfo->Count >= KEY_SET_SHORT_TIME + 1)
//     {
//       KeyInfo->Status = UP;
//       KeyInfo->last_Status = UP;
//     }
//     else if(KeyInfo->Count >= KEY_SET_SHORT_TIME)
//     {
//       KeyInfo->Status = RELAX;
//       KeyInfo->last_Status = RELAX;
//     }
//   }
//   /* the keyboard status is key down */
//   else if(KEY_DOWN == KeyInfo->KEY_PRESS)
//   {
//     /* last keyboard status is key up */
//     if(KEY_DOWN != KeyInfo->last_Status)
//       KeyInfo->Count++;
//     /* last keyboard status is key down */
//     else
//       KeyInfo->Count = 0;

//     /* update the keyboard status according the judgement count */
//     if(KeyInfo->Count >= KEY_SET_LONG_TIME)
//     {
//       KeyInfo->Status = DOWN;
//       KeyInfo->last_Status = DOWN;
//     }
//     else if(KeyInfo->Count >= KEY_SET_SHORT_TIME + 1)
//     {
//       KeyInfo->Status = SHORT_DOWN;
//       KeyInfo->last_Status = SHORT_DOWN;
//     }
//     else if(KeyInfo->Count >= KEY_SET_SHORT_TIME)
//     {
//       KeyInfo->Status = PRESS;
//       KeyInfo->last_Status = PRESS;
//     }
//   }
// }
// //------------------------------------------------------------------------------


// bool Key_W(void)
// {
//   bool res = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.W,KeyBoard_W);

//   switch (KeyBoard_Info.W.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//     break;

//     case SHORT_DOWN:
//      res = true;
//     break;

//     case DOWN:
//      res = true;
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return res;
// }

// bool Key_S(void)
// {
//   bool res = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.S,KeyBoard_S);

//   switch (KeyBoard_Info.S.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//     break;

//     case SHORT_DOWN:
//      res = true;
//     break;

//     case DOWN:
//      res = true;
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return res;
// }

// bool Key_Q(void)
// {
//   static bool dirction = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.Q,KeyBoard_Q);

//   switch (KeyBoard_Info.Q.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//       dirction = !dirction;
//     break;

//     case SHORT_DOWN:
//     break;

//     case DOWN:
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return dirction;
// }

// bool Key_F(void)
// {
//   static bool slip = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.F,KeyBoard_F);

//   switch (KeyBoard_Info.F.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//       slip = !slip;
//     break;

//     case SHORT_DOWN:
//     break;

//     case DOWN:
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return slip;
// }

// bool Key_C(void)
// {
//   bool disable = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.C,KeyBoard_C);

//   switch (KeyBoard_Info.C.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//     break;

//     case SHORT_DOWN:
//       disable = true;
//     break;

//     case DOWN:
//       disable = true;
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return disable;
// }

// bool Key_Z(void)
// {
//   bool momentum_init = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.Z,KeyBoard_Z);

//   switch (KeyBoard_Info.Z.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//     break;

//     case SHORT_DOWN:
//       momentum_init = true;
//     break;

//     case DOWN:
//       momentum_init = true;
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return momentum_init;
// }

// bool Key_Shift(void)
// {
//   bool res = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.SHIFT,KeyBoard_SHIFT);

//   switch (KeyBoard_Info.SHIFT.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//     break;

//     case SHORT_DOWN:
//      res = true;
//     break;

//     case DOWN:
//      res = true;
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return res;
// }

// bool Key_Ctrl(void)
// {
//   static bool Front_Side = false;

//   /* update the key status */
//   Key_Status_Update(&KeyBoard_Info.CTRL,KeyBoard_CTRL);

//   switch (KeyBoard_Info.CTRL.Status)
//   {
//     case UP:
//     break;

//     case PRESS:
//       Front_Side = !Front_Side;
//     break;

//     case SHORT_DOWN:
//     break;

//     case DOWN:
//     break;

//     case RELAX:
//     break;

//     default:break;
//   }
//   return Front_Side;
// }
