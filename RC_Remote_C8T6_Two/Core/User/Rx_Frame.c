//
// Created by keeper on 2026/7/30.
//

#include "Rx_Frame.h"

uint8_t Rx_Buffer[RX_BUFFER_SIZE];
Rx_Frame RFrame = {0};

void Uart_Start(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

static void Rx_FrameProcess(uint8_t* Data, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        uint8_t Byte = Data[i];
        switch (RFrame.State)
        {
        case RX_STATE_IDLE:
            {
                if (Byte == RX_FRAME_HEADER)
                {
                    RFrame.State = RX_STATE_DATA;
                    RFrame.Index = 0;
                }
                break;
            }
        case RX_STATE_DATA:
            {
                if (Byte == RX_FRAME_HEADER)
                {
                    RFrame.Index = 0;
                }
                else
                {
                    RFrame.Value[RFrame.Index++] = Byte;
                    if (RFrame.Index == RX_FRAME_LEN)
                    {
                        RFrame.State = RX_STATE_TAIL;
                    }
                }
                break;
            }
        case RX_STATE_TAIL:
            {
                if (Byte == RX_FRAME_TAIL)
                {
                    RFrame.Flag = 1;
                }
                else if (Byte == RX_FRAME_HEADER)
                {
                    RFrame.Index = 0;
                    RFrame.State = RX_STATE_DATA;
                    break;
                }
                RFrame.State = RX_STATE_IDLE;
                RFrame.Index = 0;
                break;
            }
        default:
            {
                RFrame.State = RX_STATE_IDLE;
                RFrame.Index = 0;
            }

        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        Rx_FrameProcess(Rx_Buffer, Size);
        Uart_Start(huart);
    }
}