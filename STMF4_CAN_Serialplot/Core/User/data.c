/**
 * @file    data.c
 * @brief   全局数据定义
 * @version 1.0
 * @date    2026-07-19
 *
 * 集中定义所有模块的全局变量、结构体实例、缓冲区。
 * extern 声明在各模块头文件中。
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "drv_can.h"
#include "SerialPlot.h"
#include "Uart.h"

/* ===================== CAN 数据定义 ===================== */

/** CAN 发送数据缓冲区 */
uint8_t CAN1_0x1ff_Tx_Data[8];
uint8_t CAN1_0x200_Tx_Data[8];
uint8_t CAN2_0x1ff_Tx_Data[8];
uint8_t CAN2_0x200_Tx_Data[8];

/** CAN 管理对象 */
Struct_CAN_Manage_Object CAN1_Manage_Object;
Struct_CAN_Manage_Object CAN2_Manage_Object;

/* ===================== UART 数据定义 ===================== */

/** UART 管理对象 */
Struct_UART_Manage_Object UART2_Struct = {0};
Struct_UART_Manage_Object UART3_Struct = {0};

Struct_Serialplot Serialplot2 = {0};

uint8_t UART2_Tx_Buffer[UART_TX_BUFFER_SIZE];
uint8_t UART3_Tx_Buffer[UART_TX_BUFFER_SIZE];

uint8_t UART2_Rx_Buffer[UART_RX_BUFFER_SIZE];
uint8_t UART3_Rx_Buffer[UART_RX_BUFFER_SIZE];
