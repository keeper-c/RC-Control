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

            Rx_Frame.Key_State[0]   = buf[i + 1];
            Rx_Frame.Key_State[1]   = buf[i + 2];
            Rx_Frame.Key_State[2]   = buf[i + 3];
            Rx_Frame.Slider_Value[0] = buf[i + 4];
            Rx_Frame.Slider_Value[1] = buf[i + 5];
            Rx_Frame.Battery_Value   = buf[i + 6];

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
        }

        HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}