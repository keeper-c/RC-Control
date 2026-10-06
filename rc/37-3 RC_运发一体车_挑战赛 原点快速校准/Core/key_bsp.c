#include "main.h"

// 按键GPIO中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
    }
}

extern uint16_t para_speed; // 速度矫正常量, 线性补偿
extern uint8_t disc_rx[28];
extern uint8_t debug_data[2];
static char ch_uart2[20];
extern float distance_Gu;
// 串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        para_speed = ((debug_data[0] >> 4) * 1000) + ((debug_data[0] & 0x0f) * 100) + ((debug_data[1] >> 4) * 10) + (debug_data[1] & 0x0f);

        sprintf(ch_uart2, "speed:%d", para_speed);

        HAL_UART_Transmit_IT(&huart2, (uint8_t *)ch_uart2, sizeof(ch_uart2));
        HAL_UART_Receive_IT(&huart2, debug_data, 2);
    }
    if (huart->Instance == UART7)
    {
        for (uint8_t i = 0; i < 28; i++)
            Data_Analyse(disc_rx[i]);
        HAL_UART_Receive_IT(&huart7, disc_rx, 28);
    }
}

// 遥控器接收    // 串口接收空闲中断
extern DMA_HandleTypeDef hdma_usart1_rx;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {
        if (Size == 26) // Elrs遥控器
            ELRSDataProcess(Elrs.rdata);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Elrs.rdata, 27);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }

    if (huart->Instance == USART6)
    {
        vision.flag = 1;
        vision.xy[0] = ((vision.data[0] << 8) + vision.data[1] - 540 - 5);
        vision.xy[1] = ((vision.data[2] << 8) + vision.data[3] - 360);
        HAL_UARTEx_ReceiveToIdle_IT(&huart6, vision.data, 5);
        TIM13->CNT = 0;
    }
}
