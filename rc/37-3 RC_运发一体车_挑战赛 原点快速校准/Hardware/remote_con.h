#ifndef __REMOTE_CON_H
#define __REMOTE_CON_H
#include "stdint.h"

typedef struct
{
    uint8_t remote_data[20]; // 遥控器数据原始值存放

    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;

    uint8_t s1;
    uint8_t s2;

    uint16_t v; // 非通道控制值
} Remote_value;

typedef struct
{
    uint8_t rdata[28];   // 遥控器数据原始值存放
    int16_t ch[11];      // 各通道的值
    uint8_t ch_flag[10]; // 5~9通道的数据改变标志位
    uint8_t ch_last[10]; // 记录上一次的通道值，用于检测数据改变，得到标志位
} ELRS;

extern Remote_value RC_Ctrl;
extern ELRS Elrs;

void RemoteDataProcess(uint8_t *pData);
void ELRSDataProcess(uint8_t *rdata);
#endif
/* 小白遥控器初始化代码 */
//    extern Remote_value RC_Ctrl;;
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RC_Ctrl.remote_data, 20);

/* Elrs遥控器初始化代码 */
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Elrs.rdata, 27);

/* 遥控器回调函数部分（只选用一个，你也可以都用）*/
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
////    if(huart == &huart1 && Size == 18)    // 小白遥控器
////    {
////        RemoteDataProcess(RC_Ctrl.remote_data);
////        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,RC_Ctrl.remote_data,20);
////        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
////    }

//    if(huart == &huart6 && Size == 26)  // Elrs遥控器
//    {
//        ELRSDataProcess(Elrs.rdata);
//    }
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Elrs.rdata, 27);
//    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
//}
