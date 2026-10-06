/**
 * @file drv_can.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief CAN通信初始化与配置
 * @version 0.2
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 */

/* Includes ------------------------------------------------------------------*/

#include "drv_can.h"
#include "data.h"

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

static void CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan, uint32_t Fifo);

/* function prototypes -------------------------------------------------------*/

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function)
{
    Struct_CAN_Manage_Object *obj = (Struct_CAN_Manage_Object *)Peripheral_Get_Struct(hcan->Instance);
    if (obj == NULL) return;

    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    obj->CAN_Handler = hcan;
    obj->Callback_Function = Callback_Function;
}

void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    can_filter_init_structure.FilterIdHigh = (ID & 0x7FF) << 5;
    can_filter_init_structure.FilterIdLow = 0x0000;
    can_filter_init_structure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
    can_filter_init_structure.FilterMaskIdLow = 0x0000;
    can_filter_init_structure.FilterBank = (Object_Para >> 3) & 0x1F;
    can_filter_init_structure.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_init_structure.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_init_structure.FilterActivation = ENABLE;
    can_filter_init_structure.SlaveStartFilterBank = 14;
    can_filter_init_structure.FilterFIFOAssignment = (Object_Para >> 2) & 0x01;

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_structure);
}

uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    assert_param(hcan != NULL);

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

static void CAN_RxMsgPendingCallback(CAN_HandleTypeDef *hcan, uint32_t Fifo)
{
    Struct_CAN_Manage_Object *obj = (Struct_CAN_Manage_Object *)Peripheral_Get_Struct(hcan->Instance);
    if (obj == NULL) return;

    HAL_CAN_GetRxMessage(hcan, Fifo, &obj->Rx_Buffer.Header, obj->Rx_Buffer.Data);
    if (obj->Callback_Function)
        obj->Callback_Function(&obj->Rx_Buffer);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxMsgPendingCallback(hcan, CAN_FILTER_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxMsgPendingCallback(hcan, CAN_FILTER_FIFO1);
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/