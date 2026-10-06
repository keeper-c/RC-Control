/**
 * @file My_CAN.c
 * @brief CAN总线通信驱动实现文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 实现CAN总线的初始化、滤波器配置、数据发送和接收回调功能
 */

#include "drv_can.h"

/* 全局变量定义 --------------------------------------------------------*/

Struct_CAN_Manage_Object CAN1_Struct = {0};  /**< CAN1管理结构体 */
Struct_CAN_Manage_Object CAN2_Struct = {0};  /**< CAN2管理结构体 */


/* 函数定义 ----------------------------------------------------------------*/

/**
 * @brief CAN总线初始化
 * @param hcan CAN句柄指针
 * @param Callback_Function 接收回调函数
 *
 * 启动CAN总线并使能接收中断
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Callback Callback_Function)
{
    HAL_CAN_Start(hcan);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    Struct_CAN_Manage_Object *pcan = NULL;

    switch ((uint32_t)hcan->Instance)
    {
        case (uint32_t)CAN1:
        {
            pcan = &CAN1_Struct;
            break;
        }
        case (uint32_t)CAN2:
        {
            pcan = &CAN2_Struct;
            break;
        }
        default:
        {
            return;
        }
    }

    pcan->CAN_Handler = hcan;
    pcan->Callback_Function = Callback_Function;
}

/**
 * @brief CAN滤波器掩码配置
 * @param hcan CAN句柄指针
 * @param Object_Para 滤波器参数
 *        - [7:3] 滤波器编号(0-27)
 *        - [2] FIFO选择(0=FIFO0, 1=FIFO1)
 *        - [1] ID类型(0=标准帧, 1=扩展帧,当前仅支持标准帧)
 *        - [0] 帧类型(0=数据帧, 1=遥控帧,当前仅支持数据帧)
 * @param ID 滤波器ID
 * @param Mask_ID 滤波器掩码
 *
 * 配置CAN滤波器以接收特定ID的CAN消息
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
    CAN_FilterTypeDef can_filter_init_structure;

    if (Object_Para & 0x01)
    {
        return;
    }

    if ((Object_Para & 0x02) >> 1)
    {
        return;
    }

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

/**
 * @brief CAN数据发送
 * @param hcan CAN句柄指针
 * @param ID CAN标准ID
 * @param Data 发送数据缓冲区指针
 * @param Length 数据长度(1-8字节)
 * @return HAL_StatusTypeDef 执行状态
 *
 * 使用CAN标准数据帧发送数据
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t used_mailbox;

    tx_header.StdId = ID;
    tx_header.ExtId = 0;
    tx_header.IDE = 0;
    tx_header.RTR = 0;
    tx_header.DLC = Length;

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &used_mailbox));
}

/**
 * @brief CAN接收FIFO0中断回调
 * @param hcan CAN句柄指针
 *
 * HAL库CAN接收FIFO0消息pending中断回调函数
 * 当CAN FIFO0接收到新消息时由HAL库自动调用
 *
 * 处理流程:
 * 1. 根据CAN实例(CAN1/CAN2)获取对应的管理结构体
 * 2. 从FIFO0读取消息头和数据到临时缓冲区
 * 3. 调用用户注册的回调函数处理接收到的数据
 *
 * @note 该函数为HAL库弱函数重写，必须保证全工程唯一实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static Struct_CAN_Rx_Buffer can_rx_buffer;
    Struct_CAN_Manage_Object *pcan = NULL;

    switch ((uint32_t)hcan->Instance)
    {
        case (uint32_t)CAN1:
        {
            pcan = &CAN1_Struct;
            break;
        }
        case (uint32_t)CAN2:
        {
            pcan = &CAN2_Struct;
            break;
        }
        default:
        {
            return;
        }
    }

    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can_rx_buffer.Header, can_rx_buffer.Data);

    if (pcan->Callback_Function != NULL)
    {
        pcan->Callback_Function(&can_rx_buffer);
    }
}

/**
 * @brief CAN接收FIFO1中断回调
 * @param hcan CAN句柄指针
 *
 * HAL库CAN接收FIFO1消息pending中断回调函数
 * 当CAN FIFO1接收到新消息时由HAL库自动调用
 *
 * 处理流程:
 * 1. 根据CAN实例(CAN1/CAN2)获取对应的管理结构体
 * 2. 从FIFO1读取消息头和数据到临时缓冲区
 * 3. 调用用户注册的回调函数处理接收到的数据
 *
 * @note 该函数为HAL库弱函数重写，必须保证全工程唯一实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static Struct_CAN_Rx_Buffer can_rx_buffer;
    Struct_CAN_Manage_Object *pcan = NULL;

    switch ((uint32_t)hcan->Instance)
    {
        case (uint32_t)CAN1:
        {
            pcan = &CAN1_Struct;
            break;
        }
        case (uint32_t)CAN2:
        {
            pcan = &CAN2_Struct;
            break;
        }
        default:
        {
            return;
        }
    }

    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &can_rx_buffer.Header, can_rx_buffer.Data);

    if (pcan->Callback_Function != NULL)
    {
        pcan->Callback_Function(&can_rx_buffer);
    }
}
