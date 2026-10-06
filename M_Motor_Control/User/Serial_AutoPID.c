#include "Serial_AutoPID.h"

Serial_AutoPID_t g_AutoPID;

void Serial_AutoPID_Init(Serial_AutoPID_t *obj, UART_HandleTypeDef *huart, uint8_t Handle_Handle)
{
    memset(obj, 0, sizeof(Serial_AutoPID_t));
    obj->huart = huart;   
    obj->Frame_Handle = Handle_Handle;
    HAL_UARTEx_ReceiveToIdle_DMA(obj->huart, obj->rx_buf, UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(obj->huart->hdmarx, DMA_IT_HT);
}

void Serial_AutoPID_Send(Serial_AutoPID_t *obj, DJ_Motor_t *motor)
{
    float msg[2];
    msg[0] = motor->speed_pid.target;
    msg[1] = motor->RX_Frame.omega_out_rad_s;
    HAL_UART_Transmit(obj->huart, (uint8_t *)&obj->Frame_Handle, 1, HAL_MAX_DELAY);
    HAL_UART_Transmit(obj->huart, (uint8_t *)&msg, sizeof(msg), HAL_MAX_DELAY);
}

/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != g_AutoPID.huart) return;

    if (Size == 8 && g_AutoPID.rx_buf[0] == 0xAA && g_AutoPID.rx_buf[7] == 0xFF)
    {
        g_AutoPID.Kp = (float)((g_AutoPID.rx_buf[1] << 8) | g_AutoPID.rx_buf[2]);
        g_AutoPID.Ki = (float)((g_AutoPID.rx_buf[3] << 8) | g_AutoPID.rx_buf[4]);
        g_AutoPID.Kd = (float)((g_AutoPID.rx_buf[5] << 8) | g_AutoPID.rx_buf[6]);
        g_AutoPID.pid_ready = 1;
    }
    else if (Size == 6 && g_AutoPID.rx_buf[0] == 0xCC && g_AutoPID.rx_buf[5] == 0xFF)
    {
        memcpy(&g_AutoPID.target, &g_AutoPID.rx_buf[1], 4);
        g_AutoPID.target_ready = 1;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(g_AutoPID.huart, g_AutoPID.rx_buf, UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(g_AutoPID.huart->hdmarx, DMA_IT_HT);
}
    */
   