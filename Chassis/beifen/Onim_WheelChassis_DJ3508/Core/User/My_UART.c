#include "My_UART.h"

Struct_UART_Manage_Object UART2_Struct = {0};
Struct_UART_Manage_Object UART3_Struct = {0};

uint8_t rx_data[UART_RX_BUFFER_SIZE];
uint8_t tx_data[UART_TX_BUFFER_SIZE];

static uint8_t Uart_DMA_Buf[UART_DMA_BUF_SIZE];
Uart_RxFrame_t Uart_RxFrame = {0};

void UART_Init(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t tx_length,
               uint8_t *rx_data, uint16_t rx_length, UART_RX_Callback callback_function)
{
    Struct_UART_Manage_Object *puart = NULL;

    switch ((uint32_t)huart->Instance)
    {
        case (uint32_t)USART2:
        {
            puart = &UART2_Struct;
            break;
        }
        case (uint32_t)USART3:
        {
            puart = &UART3_Struct;
            break;
        }
        default:
        {
            return;
        }
    }

    puart->UART_HANDLE = huart;
    puart->UART_Tx_Data = tx_data;
    puart->Tx_Length = tx_length;
    puart->UART_Rx_Data = rx_data;
    puart->Rx_Length = rx_length;
    puart->CallBack_Function = callback_function;

    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(huart, data, length));
}

void TIM_UART_PeriodElapsedCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2)
    {
        UART_Send_Data(&huart2, UART2_Struct.UART_Tx_Data, 1 + size * (uint16_t)sizeof(float));
    }
    else if (huart->Instance == USART3)
    {
        //UART_Send_Data(&huart3, UART3_Struct.UART_Tx_Data, 1 + size * (uint16_t)sizeof(float));
    }
}

static void Uart_ParseFrame(const uint8_t *data, uint16_t size)
{
    if (size < UART_FRAME_SIZE)
    {
        return;
    }

    for (uint16_t i = 0; i <= size - UART_FRAME_SIZE; i++)
    {
        if (data[i] == 0xAA && data[i + 4] == 0xBB)
        {
            Uart_RxFrame.X     = (int8_t)data[i + 1];
            Uart_RxFrame.Y     = (int8_t)data[i + 2];
            Uart_RxFrame.Omega = (int8_t)data[i + 3];
            Uart_RxFrame.Update = 1;
            break;
        }
    }
}

void Uart_RxCallback(uint8_t *data, uint16_t size)
{
    if (size > UART_DMA_BUF_SIZE)
    {
        size = UART_DMA_BUF_SIZE;
    }

    memcpy(Uart_DMA_Buf, data, size);
    Uart_ParseFrame(Uart_DMA_Buf, size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *puart = NULL;

    switch ((uint32_t)huart->Instance)
    {
    case (uint32_t)USART2:
        puart = &UART2_Struct;
        break;
    case (uint32_t)USART3:
        puart = &UART3_Struct;
        break;
    default:
        return;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Struct_UART_Manage_Object *puart = NULL;

    switch ((uint32_t)huart->Instance)
    {
    case (uint32_t)USART2:
        {
            puart = &UART2_Struct;
            break;
        }
    case (uint32_t)USART3:
        {
            puart = &UART3_Struct;
            break;
        }
    default:
        {
            return;
        }
    }

    if (puart->CallBack_Function != NULL)
    {
        puart->CallBack_Function(puart->UART_Rx_Data, size);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}