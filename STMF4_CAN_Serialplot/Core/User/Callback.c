//
// Created by keeper on 2026/7/18.
//

#include "Callback.h"
#include "SerialPlot.h"

void UART2_Callback(uint8_t *rx_buf, uint16_t length)
{
    Serialplot_UART_RxCpltCallback(&Serialplot2, rx_buf);
}