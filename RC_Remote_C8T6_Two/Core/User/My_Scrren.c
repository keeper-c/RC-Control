//
// Created by keeper on 2026/7/30.
//

#include "My_Scrren.h"

uint8_t Tx_Buffer[TX_BUFFER_SIZE];
uint8_t Rx_Buffer[RX_BUFFER_SIZE];

Scrren Scrren1 = {0};
KeyMap_t KeyMap[KEYNUM] =
{   {1,&Scrren1.Key_State[0]},
    {2,&Scrren1.Key_State[1]},
    {3,&Scrren1.Key_State[2]},
    {4,&Scrren1.Key_State[3]},
    {6,&Scrren1.Key_State[4]},
    {7,&Scrren1.Key_State[5]},
};

void SetValue(Scrren* scrren, uint8_t ID, uint16_t Value)
{
    for (int i = 0; i < KEYNUM; i++)
    {
        if (KeyMap[i].ID == ID)
        {
            *(uint16_t*)KeyMap[i].Value = Value;
        }
    }
}

void SendFrame(Scrren* scrren)
{
    uint8_t frame[TX_BUFFER_SIZE];

    frame[0] = TX_FRAME_HEADER;
    frame[1] = scrren->Key_State[0];
    frame[2] = scrren->Key_State[1];
    frame[3] = scrren->Key_State[2];
    frame[4] = scrren->Key_State[3];
    frame[5] = scrren->Key_State[4];
    frame[6] = scrren->Key_State[5];
    frame[7] = TX_FRAME_TAIL;

    HAL_UART_Transmit(&huart2, frame, TX_FRAME_LEN, HAL_MAX_DELAY);
}

void Uart_Start(UART_HandleTypeDef *huart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, Rx_Buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

static uint8_t Rx_Frame_Process(uint8_t* Rx, uint16_t Size)
{
    uint16_t ID = 0;
    uint16_t Value = 0;

    for (int i = 0; i < Size; i++)
    {
        if (Rx[i] == RX_FRAME_HEADER)
        {
            if (RX_FRAME_LEN + i > Size)
            {
                return 0;
            }

            if (Rx[i + RX_FRAME_LEN - 1] != RX_FRAME_TAIL)
            {
                continue;
            }

            ID = Rx[i + 2] << 8 | Rx[i + 1];
            Value = Rx[i + 4] << 8 | Rx[i + 3];

            SetValue(&Scrren1, ID, Value);

            return 1;
        }
    }

    return 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->Instance == USART2)
    {
        if (Rx_Frame_Process(Rx_Buffer, Size))
        {
            Scrren1.Rx_Flag = 1;
        }
        Uart_Start(huart);
    }

}