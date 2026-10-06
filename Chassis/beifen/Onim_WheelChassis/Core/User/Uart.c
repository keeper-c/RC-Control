#include "Uart.h"
#include "cmsis_os2.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

Uart_RxFrame_t Uart_RxFrame = {0};

static uint8_t Uart_DMA_Buf[UART_DMA_BUF_SIZE];

static void Uart_ParseFrame(const uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i <= size - UART_FRAME_SIZE; i++)
    {
        if (data[i] == 0xAA && data[i + 4] == 0xBB)
        {
            Uart_RxFrame.X     = (int8_t)data[i + 1];
            Uart_RxFrame.Y     = (int8_t)data[i + 2];
            Uart_RxFrame.Omega = (int8_t)data[i + 3];
            osMessageQueuePut(UartRx_QueueHandle, &Uart_RxFrame, 0, 0);
        }
    }
}

void Uart_Start(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Uart_DMA_Buf, UART_DMA_BUF_SIZE);
}

void Uart_RxCallback(uint16_t size)
{
    Uart_ParseFrame(Uart_DMA_Buf, size);
    Uart_Start();
}

