/**
 * @file My_UART.h
 * @brief UART通信驱动头文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 定义UART通信所需的数据结构、类型定义和函数声明
 */

#ifndef MY_UART_H
#define MY_UART_H

/* 头文件包含 ------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* 宏定义 ----------------------------------------------------------------*/
#define UART_TX_BUFFER_SIZE (256)
#define UART_RX_BUFFER_SIZE (256)

#define UART_FRAME_SIZE     5
#define UART_DMA_BUF_SIZE   32
/* 类型定义 ----------------------------------------------------------------*/

/**
 * @brief UART接收回调函数类型
 * @param rx_buf 接收数据缓冲区指针
 * @param length 实际接收到的数据长度
 *
 * 当UART接收到数据时调用此回调函数，将接收到的数据传递给应用层处理
 * 采用DMA方式接收，支持空闲中断触发接收完成
 */
typedef void (*UART_RX_Callback)(uint8_t *rx_buf, uint16_t length);

typedef struct
{
    int8_t X;
    int8_t Y;
    int8_t Omega;
} Uart_RxFrame_t;

/**
 * @brief UART管理结构体
 *
 * 管理UART总线的句柄、发送/接收缓冲区和回调函数，支持USART2和USART3双通道
 */
typedef struct
{
    UART_HandleTypeDef *UART_HANDLE;   /**< UART句柄指针，指向HAL库的UART句柄 */
    uint8_t *UART_Tx_Data;            /**< 发送数据缓冲区指针 */
    uint16_t Tx_Length;               /**< 发送数据缓冲区长度 */
    uint8_t *UART_Rx_Data;            /**< 接收数据缓冲区指针 */
    uint16_t Rx_Length;               /**< 接收数据缓冲区长度 */
    UART_RX_Callback CallBack_Function; /**< 接收回调函数指针，UART接收完成时调用 */
} Struct_UART_Manage_Object;

/* 外部变量声明 ------------------------------------------------------------*/

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern Struct_UART_Manage_Object UART2_Struct;
extern Struct_UART_Manage_Object UART3_Struct;

extern uint8_t rx_data[];
extern uint8_t tx_data[];


/* 函数声明 ------------------------------------------------------------*/

/**
 * @brief UART初始化
 * @param huart UART句柄指针
 * @param tx_data 发送数据缓冲区指针
 * @param tx_length 发送数据长度
 * @param rx_data 接收数据缓冲区指针
 * @param rx_length 接收数据长度
 * @param callback_function 接收回调函数
 */
void UART_Init(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t tx_length,
               uint8_t *rx_data, uint16_t rx_length, UART_RX_Callback callback_function);

/**
 * @brief UART数据发送
 * @param huart UART句柄指针
 * @param data 发送数据缓冲区指针
 * @param length 数据长度
 * @return uint8_t HAL状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length);

/**
 * @brief UART定时发送回调函数
 * @param huart UART句柄指针
 * @param size 数据大小
 */
void TIM_UART_PeriodElapsedCallback(UART_HandleTypeDef *huart, uint16_t size);

void Uart_RxCallback(uint8_t *data, uint16_t size);

#endif /* MY_UART_H */