/**
 * @file Uart.c
 * @brief UART通信驱动实现文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 实现UART总线的初始化、数据发送和DMA接收回调功能
 */

#include "Uart.h"

/* 内部函数声明 ---------------------------------------------------------*/
static Struct_UART_Manage_Object *UART_Get_Struct(USART_TypeDef *Instance);

/* 函数定义 ----------------------------------------------------------------*/

static Struct_UART_Manage_Object *UART_Get_Struct(USART_TypeDef *Instance)
{
    static const struct {
        USART_TypeDef *Instance;
        Struct_UART_Manage_Object *puart;
    } uart_map[] = {{USART2, &UART2_Struct}, {USART3, &UART3_Struct}};

    for (uint8_t i = 0; i < sizeof(uart_map)/sizeof(uart_map[0]); i++) {
        if (Instance == uart_map[i].Instance) {
            return uart_map[i].puart;
        }
    }
    return NULL;
}

void UART_Init(UART_HandleTypeDef *huart, uint8_t *tx_data, uint16_t tx_length,
               uint8_t *rx_data, uint16_t rx_length, UART_RX_Callback callback_function)
{
    Struct_UART_Manage_Object *puart = UART_Get_Struct(huart->Instance);
    if (puart == NULL) return;

    puart->UART_HANDLE = huart;
    puart->UART_Tx_Data = tx_data;
    puart->Tx_Length = tx_length;
    puart->UART_Rx_Data = rx_data;
    puart->Rx_Length = rx_length;
    puart->CallBack_Function = callback_function;

    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief UART数据发送
 * @param huart UART句柄指针
 * @param data 发送数据缓冲区指针
 * @param length 数据长度
 * @return uint8_t HAL状态
 *
 * 使用DMA方式发送数据
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(huart, data, length));
}

/**
 * @brief UART定时发送回调函数
 * @param huart UART句柄指针
 * @param size 数据大小
 *
 * 应在定时器中断中调用,用于周期性发送UART数据
 */
void TIM_UART_PeriodElapsedCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Struct_UART_Manage_Object *puart = UART_Get_Struct(huart->Instance);
    if (puart == NULL) return;

    UART_Send_Data(puart->UART_HANDLE, puart->UART_Tx_Data, 1 + size * (uint16_t)sizeof(float));
}

/**
 * @brief UART接收事件回调函数
 * @param huart UART句柄指针
 * @param size 实际接收到的数据大小
 *
 * HAL库UART接收事件回调函数，支持空闲中断和接收完成中断触发
 * 采用DMA方式接收，支持不定长数据接收（通过空闲中断判断帧结束）
 * 
 * 处理流程:
 * 1. 根据UART实例(USART2/USART3)获取对应的管理结构体
 * 2. 调用用户注册的回调函数处理接收到的数据
 * 3. 重新启动DMA接收，等待下一次数据
 * 
 * @note 该函数为HAL库弱函数重写，必须保证全工程唯一实现
 * @note 每次接收完成后必须重新启动DMA接收，否则后续数据将丢失
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Struct_UART_Manage_Object *puart = UART_Get_Struct(huart->Instance);
    if (puart == NULL) return;

    if (puart->CallBack_Function != NULL)
    {
        puart->CallBack_Function(puart->UART_Rx_Data, size);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(puart->UART_HANDLE, puart->UART_Rx_Data, puart->Rx_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}