#include "My_UART.h"
#include "cmsis_os2.h"

Struct_UART_Manage_Object UART2_Struct = {0};
Struct_UART_Manage_Object UART3_Struct = {0};

uint8_t rx_data[UART_RX_BUFFER_SIZE];
uint8_t tx_data[UART_TX_BUFFER_SIZE];

typedef enum {
    STATE_IDLE,
    STATE_DATA,
    STATE_TAIL,
} RxState_t;

static RxState_t rx_state = STATE_IDLE;
static uint8_t  rx_buf[6];
static uint8_t  rx_idx;

static Struct_UART_Manage_Object* GetUartStruct(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) return &UART2_Struct;
    if (huart->Instance == USART3) return &UART3_Struct;
    return NULL;
}

static void RestartDMA(Struct_UART_Manage_Object *puart)
{
    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(puart->UART_HANDLE->hdmarx, DMA_IT_HT);
}

void UART_Init(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t tx_length,
               uint8_t *rx_data, uint16_t rx_length, UART_RX_Callback callback_function)
{
    Struct_UART_Manage_Object *puart = GetUartStruct(huart);
    if (puart == NULL) return;

    puart->UART_HANDLE = huart;
    puart->UART_Tx_Data = tx_data;
    puart->Tx_Length = tx_length;
    puart->UART_Rx_Data = rx_data;
    puart->Rx_Length = rx_length;
    puart->CallBack_Function = callback_function;

    rx_state = STATE_IDLE;
    rx_idx = 0;

    __HAL_UART_CLEAR_IDLEFLAG(huart);

    RestartDMA(puart);
}

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return HAL_UART_Transmit_DMA(huart, data, length);
}

void TIM_UART_PeriodElapsedCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Struct_UART_Manage_Object *puart = GetUartStruct(huart);
    if (puart == NULL) return;

    UART_Send_Data(puart->UART_HANDLE, puart->UART_Tx_Data, 1 + size * (uint16_t)sizeof(float));
}

void Uart_RxCallback(uint8_t *data, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        uint8_t byte = data[i];

        switch (rx_state)
        {
        case STATE_IDLE:
            if (byte == 0xAA)
            {
                rx_state = STATE_DATA;
                rx_idx = 0;
            }
            break;

        case STATE_DATA:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= 6)
            {
                rx_state = STATE_TAIL;
            }
            break;

        case STATE_TAIL:
            if (byte == 0xFF)
            {
                Uart_RxFrame_t frame;
                frame.Key_State[0]    = rx_buf[0];
                frame.Key_State[1]    = rx_buf[1];
                frame.Key_State[2]    = rx_buf[2];
                frame.Key_State[3]    = rx_buf[3];
                frame.Slider_Value[0] = rx_buf[4];
                frame.Slider_Value[1] = rx_buf[5];

                if (osMessageQueuePut(UartRx_QueueHandle, &frame, 0, 0) != osOK)
                {
                    osMessageQueueReset(UartRx_QueueHandle);
                }
            }
            rx_state = STATE_IDLE;
            rx_idx = 0;
            break;

        default:
            rx_state = STATE_IDLE;
            break;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *puart = GetUartStruct(huart);
    if (puart == NULL) return;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
        (void)huart->Instance->DR;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_FE))
    {
        __HAL_UART_CLEAR_FEFLAG(huart);
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_NE))
    {
        __HAL_UART_CLEAR_NEFLAG(huart);
    }

    rx_state = STATE_IDLE;
    rx_idx = 0;

    RestartDMA(puart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Struct_UART_Manage_Object *puart = GetUartStruct(huart);
    if (puart == NULL) return;

    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    if (puart->CallBack_Function != NULL)
    {
        puart->CallBack_Function(puart->UART_Rx_Data, size);
    }

    RestartDMA(puart);
}