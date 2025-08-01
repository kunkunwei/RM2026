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
#include "usart.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
	.rc_lost = true,
	.online_cnt = 0xFAU,
};

/**
 * @brief remote control usart RxDMA MultiBuffer
 */
uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];

/**
 * @brief 上位机通信数据结构
 */
PC_Data_Typedef pc_data = {
    .yaw_angle = 0.0f,
    .pitch_angle = 0.0f,
    .data_valid = false,
    .packet_id = 0,
    .last_update = 0,
};

/**
 * @brief 上位机通信接收缓冲区
 */
uint8_t PC_MultiRx_Buf[2][PC_RX_BUF_SIZE];

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

}

const Remote_Info_Typedef *get_remote_control_point(){
  return &remote_ctrl;
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

/**
  * @brief  解析上位机数据包
  * @param  pc_buf: 指向接收缓冲区的指针
  * @param  size: 接收到的数据长度
  * @param  pc_data: 指向上位机数据结构的指针
  * @retval none
  */
void PC_Packet_Parse(volatile const uint8_t *pc_buf, uint16_t size, PC_Data_Typedef *pc_data)
{
    if (pc_buf == NULL || pc_data == NULL || size < PC_MIN_PACKET_SIZE) {
        return;
    }

    // 查找包头
    for (uint16_t i = 0; i <= size - PC_MIN_PACKET_SIZE; i++) {
        if (pc_buf[i] != PC_PACKET_HEADER) {
            continue;
        }

        // 检查剩余字节数是否足够
        if (i + 3 >= size) {
            break;
        }

        uint8_t id = pc_buf[i + 1];
        uint8_t len = pc_buf[i + 2];

        // 检查数据长度是否合理（至少需要8字节：两个通道×4字节）
        if (len > PC_MAX_DATA_LEN || len < 8) {
            continue;
        }

        // 检查完整包是否在缓冲区内
        if (i + 3 + len + 1 > size) {
            break;
        }

        // 计算校验和
        uint8_t calc_checksum = PC_PACKET_HEADER + id + len;
        for (uint8_t j = 0; j < len; j++) {
            calc_checksum += pc_buf[i + 3 + j];
        }
        calc_checksum &= 0xFF;

        uint8_t recv_checksum = pc_buf[i + 3 + len];

        // 校验成功，解析数据
        if (calc_checksum == recv_checksum) {
            pc_data->packet_id = id;

            // 解析前两个通道数据（小端序）
            uint32_t yaw_raw = ((uint32_t)pc_buf[i + 3]) |
                              ((uint32_t)pc_buf[i + 4] << 8) |
                              ((uint32_t)pc_buf[i + 5] << 16) |
                              ((uint32_t)pc_buf[i + 6] << 24);

            uint32_t pitch_raw = ((uint32_t)pc_buf[i + 7]) |
                                ((uint32_t)pc_buf[i + 8] << 8) |
                                ((uint32_t)pc_buf[i + 9] << 16) |
                                ((uint32_t)pc_buf[i + 10] << 24);

            // 将原始数据转换为浮点数
            pc_data->yaw_angle = *((float*)&yaw_raw);
            pc_data->pitch_angle = *((float*)&pitch_raw);
            pc_data->data_valid = true;
            pc_data->last_update = HAL_GetTick();

            return; // 解析成功，退出
        }
    }
}

/**
  * @brief  发送电机实时角度值给上位机
  * @param  yaw_angle: YAW轴实时角度
  * @param  pitch_angle: PITCH轴实时角度
  * @param  packet_id: 数据包ID（此参数保留但不使用，固定使用0x01）
  * @retval none
  */
void PC_Send_Motor_Angles(float yaw_angle, float pitch_angle, uint8_t packet_id)
{
    uint8_t tx_buf[PC_TX_BUF_SIZE];
    uint8_t data_len = PC_TX_DATA_LEN;

    // 构建数据包
    tx_buf[0] = PC_TX_PACKET_HEADER;   // 包头 0x42
    tx_buf[1] = PC_TX_PACKET_ID;       // 固定ID为0x01
    tx_buf[2] = data_len;              // 数据长度

    // 打包角度数据（小端序）
    uint32_t yaw_raw = *((uint32_t*)&yaw_angle);
    uint32_t pitch_raw = *((uint32_t*)&pitch_angle);

    tx_buf[3] = (uint8_t)(yaw_raw & 0xFF);
    tx_buf[4] = (uint8_t)((yaw_raw >> 8) & 0xFF);
    tx_buf[5] = (uint8_t)((yaw_raw >> 16) & 0xFF);
    tx_buf[6] = (uint8_t)((yaw_raw >> 24) & 0xFF);

    tx_buf[7] = (uint8_t)(pitch_raw & 0xFF);
    tx_buf[8] = (uint8_t)((pitch_raw >> 8) & 0xFF);
    tx_buf[9] = (uint8_t)((pitch_raw >> 16) & 0xFF);
    tx_buf[10] = (uint8_t)((pitch_raw >> 24) & 0xFF);

    // 计算校验和：包头 + ID + 长度 + 所有数据字节之和 的低8位
    uint8_t checksum = PC_TX_PACKET_HEADER + PC_TX_PACKET_ID + data_len;
    for (uint8_t i = 0; i < data_len; i++) {
        checksum += tx_buf[3 + i];
    }
    tx_buf[3 + data_len] = checksum & 0xFF;

    // 发送数据包
    HAL_UART_Transmit(&huart6, tx_buf, 3 + data_len + 1, HAL_MAX_DELAY);
}
