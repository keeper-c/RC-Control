//
// Created by keeper on 2026/7/23.
//
#include "Types.h"

void Start_Uart_Task(void *argument)
{
    UART_Init(&huart2, tx_data, UART_TX_BUFFER_SIZE, rx_data, UART_RX_BUFFER_SIZE, Uart_RxCallback);
    for(;;)
    {
        osDelay(1);
    }
}