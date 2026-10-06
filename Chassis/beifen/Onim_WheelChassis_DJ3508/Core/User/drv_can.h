#ifndef DRV_CAN_H
#define DRV_CAN_H

/**
 * @file My_CAN.h
 * @brief CAN总线通信驱动头文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 定义CAN总线通信所需的数据结构、宏定义和函数声明
 */

/* 头文件包含 ------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 宏定义 ----------------------------------------------------------------*/

/**
 * @brief CAN滤波器配置宏
 * @param x 滤波器编号(0-27)
 */
#define CAN_FILTER(x) ((x) << 3)

/**
 * @brief CAN FIFO选择宏
 */
#define CAN_FIFO_0 (0 << 2)  /**< 选择FIFO0 */
#define CAN_FIFO_1 (1 << 2)  /**< 选择FIFO1 */

/**
 * @brief CAN ID类型宏
 */
#define CAN_STDID (0 << 1)  /**< 标准帧 */
#define CAN_EXTID (1 << 1)   /**< 扩展帧 */

/**
 * @brief CAN帧类型宏
 */
#define CAN_DATA_TYPE (0 << 0)   /**< 数据帧 */
#define CAN_REMOTE_TYPE (1 << 0) /**< 遥控帧 */

/* 类型定义 ----------------------------------------------------------------*/

/**
 * @brief CAN接收数据结构体
 *
 * 用于存储从CAN总线接收到的消息，包含消息头和数据部分
 */
typedef struct
{
    CAN_RxHeaderTypeDef Header;  /**< CAN接收消息头，包含ID、DLC、帧类型等信息 */
    uint8_t Data[8];             /**< CAN接收数据，最多8字节 */
} Struct_CAN_Rx_Buffer;

/**
 * @brief CAN接收回调函数类型
 * @param Rx_Buffer CAN接收数据结构体指针
 *
 * 当CAN接收到消息时调用此回调函数，将接收到的数据传递给应用层处理
 */
typedef void (*CAN_Callback)(Struct_CAN_Rx_Buffer *);

/**
 * @brief CAN通信管理结构体
 *
 * 管理CAN总线的句柄和回调函数，支持CAN1和CAN2双通道
 */
typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;  /**< CAN句柄指针，指向HAL库的CAN句柄 */
    CAN_Callback Callback_Function;  /**< 接收回调函数指针，CAN接收中断触发时调用 */
} Struct_CAN_Manage_Object;

/* 外部变量声明 ------------------------------------------------------------*/

extern CAN_HandleTypeDef hcan1;
/* extern CAN_HandleTypeDef hcan2; */

extern Struct_CAN_Manage_Object CAN1_Struct;
extern Struct_CAN_Manage_Object CAN2_Struct;


/* 函数声明 ------------------------------------------------------------*/

/**
 * @brief CAN总线初始化
 * @param hcan CAN句柄指针
 * @param Callback_Function 接收回调函数
 *
 * 启动CAN总线并使能接收中断
 */
void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Callback Callback_Function);

/**
 * @brief CAN滤波器掩码配置
 * @param hcan CAN句柄指针
 * @param Object_Para 滤波器参数
 * @param ID 滤波器ID
 * @param Mask_ID 滤波器掩码
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief CAN数据发送
 * @param hcan CAN句柄指针
 * @param ID CAN标准ID
 * @param Data 发送数据缓冲区指针
 * @param Length 数据长度(1-8字节)
 * @return uint8_t 执行状态
 */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);


#endif
