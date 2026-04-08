#include "InterfaceCAN.h"


static BSP::InterfaceCAN *CANDeviceList[MAX_CAN_DEVICES];
static uint8_t CANDeviceCount;

static void CAN_config_init(){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

namespace BSP{

    //该函数在main函数初始化HAL库后开始时被调用，用于初始化具体的CAN设备
    void CAN_UserInit(){

        CAN_config_init();

        for (int i = 0; i < CANDeviceCount; ++i) {
            CANDeviceList[i]->init();
//            HAL_Delay(200);
        }

    }

    //该函数在InterfaceCAN的构造函数中被调用，用于注册CAN设备。
    //对于静态类，调用时机位于main函数之前，不要在这里调用HAL库相关函数
    void registerCANDevice(InterfaceCAN *canDevice){
        if(CANDeviceCount < MAX_CAN_DEVICES){
            CANDeviceList[CANDeviceCount++] = canDevice;
        }
    }

    InterfaceCAN::InterfaceCAN(CAN_HandleTypeDef *hcan,uint32_t RxId,uint32_t TxId):
    hCAN(hcan), RxStdId(RxId), TxStdId(TxId){
        registerCANDevice(this);
    }

    bool InterfaceCAN::sendMessage(uint8_t *data, uint8_t &len){

        if(len > 8) len = 8; // CAN数据长度最大为8字节
        CAN_TxHeaderTypeDef TxHeader;
        TxHeader.StdId = TxStdId;
        TxHeader.ExtId = 0;
        TxHeader.IDE = CAN_ID_STD;
        TxHeader.RTR = CAN_RTR_DATA;
        TxHeader.DLC = len;
        TxHeader.TransmitGlobalTime = DISABLE;


        uint32_t txMailbox;

        //taskENTER_CRITICAL();
        bool r = HAL_CAN_AddTxMessage(hCAN, &TxHeader, data, &txMailbox) == HAL_OK;
        //taskEXIT_CRITICAL();

        return r;
    }

    __inline bool InterfaceCAN::isMatchId(uint32_t id) const{
        return (id) == (RxStdId);
    }


}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef tmp_header;
    uint8_t tmp_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tmp_header, tmp_data);

    for (int i = 0; i < CANDeviceCount; ++i) {
        if(CANDeviceList[i]->hCAN == hcan){
            if(CANDeviceList[i]->isMatchId(tmp_header.StdId)){
                CANDeviceList[i]->receiveMessage(tmp_header.StdId, tmp_data, tmp_header.DLC);
                break;//一般认为一个ID只会被一个设备处理
            }
        }
    }


}
