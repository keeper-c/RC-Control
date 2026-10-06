/**
  * @file    drv_can.c
  * @brief   CAN驱动实现文件，提供CAN初始化、过滤器配置、数据发送和接收回调功能
  * @author  keeper
  * @date    2026/7/18
  */

#include "drv_can.h"

uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];

Struct_CAN_Manage_Object CAN1_Manage_Object;
Struct_CAN_Manage_Object CAN2_Manage_Object;


/**
  * @brief   CAN控制器初始化函数
  * @param   hcan: CAN控制器句柄指针
  * @retval  无
  * @note    启动CAN控制器并使能FIFO0和FIFO1的消息挂起中断
  */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Start(hcan);                                           /* 启动CAN控制器 */
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);        /* 使能CAN FIFO0消息挂起中断 */
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);        /* 使能CAN FIFO1消息挂起中断 */
}

/**
      * @brief   CAN过滤器掩码模式配置函数
  * @param   hcan: CAN控制器句柄指针
  * @param   Object_Para: 过滤器配置参数
  * @param   ID: 过滤器ID（11位标准ID）
  * @param   Mask: 过滤器掩码（11位）
  * @retval  无
  * @note    Object_Para位定义: bit0-1保留, bit2选择FIFO, bit3-7过滤器编号
  */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask)
{
    CAN_FilterTypeDef can_filter_init_config;                      /* CAN过滤器初始化配置结构体 */

    if(Object_Para & 0x01)                                        /* 检查bit0是否为1（保留位） */
    {
        return;                                                    /* 参数错误，直接返回 */
    }
    if((Object_Para & 0x02) >> 1)                                 /* 检查bit1是否为1（保留位） */
    {
        return;                                                    /* 参数错误，直接返回 */
    }

    /* 设置过滤器ID */
    can_filter_init_config.FilterIdHigh = (ID & 0x7FF) << 5;      /* 将11位ID左移5位存入高16位 */
    can_filter_init_config.FilterIdLow = 0x0000;                  /* 低16位清零 */

    /* 设置过滤器掩码 */
    can_filter_init_config.FilterMaskIdHigh = (Mask & 0x7FF) << 5; /* 将11位掩码左移5位存入高16位 */
    can_filter_init_config.FilterMaskIdLow = 0x0000;              /* 低16位清零 */

    /* 配置过滤器参数 */
    can_filter_init_config.FilterBank = (Object_Para >> 3) & 0x1F; /* 从Object_Para提取过滤器编号(bit3-7) */
    can_filter_init_config.FilterMode = CAN_FILTERMODE_IDMASK;     /* 设置为掩码模式 */
    can_filter_init_config.FilterScale = CAN_FILTERSCALE_32BIT;   /* 设置为32位过滤器 */
    can_filter_init_config.FilterActivation = ENABLE;              /* 使能过滤器 */

    can_filter_init_config.SlaveStartFilterBank = 14;              /* 设置从机(CAN2)过滤器起始编号为14 */

    can_filter_init_config.FilterFIFOAssignment = (Object_Para >> 2) & 0x01; /* 分配过滤器到FIFO0或FIFO1 */

    HAL_CAN_ConfigFilter(hcan, &can_filter_init_config);           /* 应用过滤器配置 */
}

/**
  * @brief   CAN数据发送函数
  * @param   hcan: CAN控制器句柄指针
  * @param   ID: 消息ID（11位标准ID）
  * @param   Data: 指向发送数据缓冲区的指针
  * @param   Length: 数据长度（0-8字节）
  * @retval  HAL状态码（HAL_OK表示成功）
  */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
    CAN_TxHeaderTypeDef tx_header;                                 /* CAN发送消息头结构体 */
    uint32_t mailbox;                                             /* 存储分配的发送邮箱 */

    tx_header.StdId = ID;                                         /* 设置标准ID */
    tx_header.ExtId = 0;                                          /* 扩展ID清零（不使用） */
    tx_header.IDE = CAN_ID_STD;                                   /* 设置为标准ID格式 */
    tx_header.RTR = CAN_RTR_DATA;                                 /* 设置为数据帧 */
    tx_header.DLC = Length;                                        /* 设置数据长度 */

    return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &mailbox)); /* 添加消息到发送邮箱并返回状态 */
}

/**
  * @brief   CAN定时发送回调函数
  * @param   无
  * @retval  无
  * @note    由定时器中断调用，周期性发送CAN1的0x1FF和0x200消息
  */
void TIM_CAN_PeriodElapsedCallback()
{
    CAN_Send_Data(&hcan1, 0x1ff, CAN1_0x1ff_Tx_Data, 8);          /* 发送CAN1 ID=0x1FF的消息 */
    CAN_Send_Data(&hcan1, 0x200, CAN1_0x200_Tx_Data, 8);          /* 发送CAN1 ID=0x200的消息 */
}

/**
  * @brief   CAN FIFO0消息挂起回调函数
  * @param   hcan: CAN控制器句柄指针
  * @retval  无
  * @note    HAL库回调函数，当FIFO0有新消息时被调用
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static Struct_CAN_Rx_Buffer can_rx_buffer;                    /* 静态接收缓冲区，避免频繁创建 */

    if (hcan->Instance == CAN1)                                   /* 判断是否为CAN1 */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can_rx_buffer.Header, can_rx_buffer.Data);
        CAN1_Manage_Object.CallBack_Function(&can_rx_buffer);      /* 调用CAN1的回调函数处理数据 */
    }
    else if (hcan->Instance == CAN2)                              /* 判断是否为CAN2 */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &can_rx_buffer.Header, can_rx_buffer.Data);
        CAN2_Manage_Object.CallBack_Function(&can_rx_buffer);      /* 调用CAN2的回调函数处理数据 */
    }
}

/**
  * @brief   CAN FIFO1消息挂起回调函数
  * @param   hcan: CAN控制器句柄指针
  * @retval  无
  * @note    HAL库回调函数，当FIFO1有新消息时被调用
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    static Struct_CAN_Rx_Buffer can_rx_buffer;                    /* 静态接收缓冲区，避免频繁创建 */

    if (hcan->Instance == CAN1)                                   /* 判断是否为CAN1 */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &can_rx_buffer.Header, can_rx_buffer.Data);
        CAN1_Manage_Object.CallBack_Function(&can_rx_buffer);      /* 调用CAN1的回调函数处理数据 */
    }
    else if (hcan->Instance == CAN2)                              /* 判断是否为CAN2 */
    {
        HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &can_rx_buffer.Header, can_rx_buffer.Data);
        CAN2_Manage_Object.CallBack_Function(&can_rx_buffer);      /* 调用CAN2的回调函数处理数据 */
    }
}