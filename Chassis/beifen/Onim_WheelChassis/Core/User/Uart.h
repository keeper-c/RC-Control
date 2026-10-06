#ifndef ONIM_WHEELCHASSIS_UART_H
#define ONIM_WHEELCHASSIS_UART_H

#include "stm32f4xx_hal.h"
#include "Usart.h"

#define UART_FRAME_SIZE     5
#define UART_DMA_BUF_SIZE   32

typedef struct
{
    int8_t X;
    int8_t Y;
    int8_t Omega;
} Uart_RxFrame_t;

void Uart_Start(void);
void Uart_RxCallback(uint16_t size);

#endif