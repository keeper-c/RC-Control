/**
  * @file    drv_can.h
  * @brief   CAN驱动头文件，定义CAN通信相关的数据结构和接口
  * @author  keeper
  * @date    2026/7/18
  */

#ifndef STMF4_CAN_MOTORCONTROL_DRV_CAN_H
#define STMF4_CAN_MOTORCONTROL_DRV_CAN_H

#include "stm32f4xx_hal.h"

/**
  * @brief   CAN过滤器位偏移宏
  * @param   x: 过滤器编号
  * @return  左移3位后的过滤器值
  */
#define CAN_FILTER(x) ((x) << 3)

/**
  * @brief   CAN接收FIFO选择宏
  */
#define CAN_FIFO_0 (0 << 2)    /* 使用FIFO 0 */
#define CAN_FIFO_1 (1 << 2)    /* 使用FIFO 1 */

/**
  * @brief   CAN标识符类型宏
  */
#define CAN_STDID (0 << 1)     /* 标准ID模式 */
#define CAN_EXTID (1 << 1)     /* 扩展ID模式 */

/**
  * @brief   CAN消息类型宏
  */
#define CAN_DATA_TYPE (0 << 0)       /* 数据帧类型 */
#define CAN_REMOTE_TYPE (1 << 0)     /* 远程帧类型 */

/**
  * @brief   CAN接收缓冲区结构体
  * @note    用于存储接收到的CAN消息头部和数据
  */
typedef struct
{
    CAN_RxHeaderTypeDef Header;  /* CAN接收消息头部 */
    uint8_t Data[8];             /* 接收到的数据(最多8字节) */
}Struct_CAN_Rx_Buffer;

/**
  * @brief   CAN消息回调函数指针类型
  * @param   Rx_Buffer: 指向接收缓冲区的指针
  */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer*);

/**
  * @brief   CAN管理对象结构体
  * @note    用于管理CAN控制器的句柄和回调函数
  */
typedef struct
{
    CAN_HandleTypeDef *CAN_Handle;       /* CAN控制器句柄 */
    CAN_Call_Back CallBack_Function;      /* 消息接收回调函数 */
}Struct_CAN_Manage_Object;

/* CAN控制器句柄声明 */
extern CAN_HandleTypeDef hcan1;  /* CAN1控制器句柄 */
extern CAN_HandleTypeDef hcan2;  /* CAN2控制器句柄 */

/* CAN管理对象声明 */
extern Struct_CAN_Manage_Object CAN1_Manage_Object;  /* CAN1管理对象 */
extern Struct_CAN_Manage_Object CAN2_Manage_Object;  /* CAN2管理对象 */

/* CAN发送数据缓冲区声明 */
extern uint8_t CAN1_0x1ff_Tx_Data[];  /* CAN1 ID=0x1FF的发送数据缓冲区 */
extern uint8_t CAN1_0x200_Tx_Data[];  /* CAN1 ID=0x200的发送数据缓冲区 */

extern uint8_t CAN2_0x1ff_Tx_Data[];  /* CAN2 ID=0x1FF的发送数据缓冲区 */
extern uint8_t CAN2_0x200_Tx_Data[];  /* CAN2 ID=0x200的发送数据缓冲区 */

void CAN_Init(CAN_HandleTypeDef *hcan);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);
void TIM_CAN_PeriodElapsedCallback(void);

#endif /* STMF4_CAN_MOTORCONTROL_DRV_CAN_H */