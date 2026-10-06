/**
 * @file drv_can.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief CAN通信初始化与配置
 * @version 0.1
 * @date 2022-08-02
 *
 * @copyright USTC-RoboWalker (c) 2022
 *
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

#define CAN_FILTER(x)   ((x) << 3)
#define CAN_FIFO_0      (0 << 2)
#define CAN_FIFO_1      (1 << 2)
#define CAN_STDID       (0 << 1)
#define CAN_DATA_TYPE   (0 << 0)

/* Exported types ------------------------------------------------------------*/

typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} Struct_CAN_Rx_Buffer;

typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

typedef struct
{
    CAN_HandleTypeDef *CAN_Handler;
    Struct_CAN_Rx_Buffer Rx_Buffer;
    CAN_Call_Back Callback_Function;
} Struct_CAN_Manage_Object;

/* Exported variables ---------------------------------------------------------*/

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;

/* Exported function declarations ---------------------------------------------*/

void CAN_Init(CAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/