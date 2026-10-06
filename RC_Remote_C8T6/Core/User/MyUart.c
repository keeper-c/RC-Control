//
// Created by keeper on 2026/7/25.
//

#include "MyUart.h"

Rx_Frame_t Rx_Frame = {0};

static uint8_t rx_buffer[RX_BUFFER_SIZE];

void MyUart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

static uint8_t ParseFrame(uint8_t *buf, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        if (buf[i] == RX_FRAME_HEADER)
        {
            if (i + RX_FRAME_LEN > len)
            {
                return 0;
            }

            if (buf[i + RX_FRAME_LEN - 1] != RX_FRAME_TAIL)
            {
                continue;
            }

            Rx_Frame.ID = (buf[i + 2] << 4) | buf[i + 1];
            Rx_Frame.Value = (buf[i + 4] << 4) | buf[i + 3];

            return 1;
        }
    }

    return 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (ParseFrame(rx_buffer, Size))
        {
            Rx_Frame.Rx_Flag = 1;
            Screen_SetValue(Rx_Frame.ID, Rx_Frame.Value);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}