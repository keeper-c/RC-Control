#include "usart_send.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;

/**
 * @brief  发送单个字节
 * @param  data: 要发送的字节
 */
void uart_send_byte(uint8_t data)
{
    uint32_t tout = 100; // 100×50ms = 5s 超时，防死锁
    while (HAL_UART_Transmit(&huart6, &data, 1, 50) != HAL_OK)
    {
        if (--tout == 0) return;
    }
}

/**
 * @brief  发送字符串（以'\0'结尾）
 * @param  str: 要发送的字符串
 */
void uart_send_string(const char *str)
{
    if (str == NULL)
        return;

    while (*str != '\0')
    {
        uart_send_byte((uint8_t)*str);
        str++;
    }
}

uint8_t Rx_Data[COMM_DATA];
void UART6_Receive_Start(void)
{  
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Rx_Data, COMM_DATA);
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
}

void UART6_DMA_Restart(void)
{   
    HAL_StatusTypeDef ret;
    
    /* 1. 强制关闭 DMA 流（直接写 CR 寄存器） */
    hdma_usart6_rx.Instance->CR &= ~DMA_SxCR_EN;
    hdma_usart6_tx.Instance->CR &= ~DMA_SxCR_EN;

    /* 2. 清除 DMA2 中断标志 */
    DMA2->LIFCR = 0x0F40U;       /* Stream 1 (RX) */
    DMA2->HIFCR = 0x0003D000U;   /* Stream 6 (TX) */

    /* 3. 完整重初始化 UART6 */
    HAL_UART_DeInit(&huart6);
    MX_USART6_UART_Init();

    /* 4. 清除 UART 残留错误标志（读 SR→读 DR 解除 ORE 锁死） */
    {
        volatile uint32_t dummy;
        dummy = huart6.Instance->SR;
        dummy = huart6.Instance->DR;
        (void)dummy;
    }

    /* 5. 重启 DMA 空闲接收 */
    ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Rx_Data, COMM_DATA);
    if (ret == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
}

uint32_t watch_dog_tick;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart6)
  { 
		osSemaphoreRelease(sem_commandHandle);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, Rx_Data, COMM_DATA);
		watch_dog_tick = osKernelGetTickCount();
    RobotProtect = 0;  // 串口恢复 → 解除保护
  }
}
