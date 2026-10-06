#include "sbus.h"
#include "usart.h"

SBUS_t sbus_data = {0};
uint8_t sbus_raw_buf[SBUS_LEN] = {0};


/**
 * @brief SBUS初始化，DMA接收，开启IDLE事件，关闭HT传输s过半中断
 * @param huart SBUS串口句柄，配置100000 8E2
 */
void SBUS_Init(SBUS_t *sbus, UART_HandleTypeDef *huart)
{
    sbus->sbus_huart = huart;
    HAL_UARTEx_ReceiveToIdle_DMA(sbus->sbus_huart, sbus_raw_buf, SBUS_LEN);
    __HAL_DMA_DISABLE_IT(sbus->sbus_huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief SBUS帧解析函数
 */
void SBUS_Decode(uint8_t *raw_buf, SBUS_t *sbus)
{
    if(raw_buf[0] != 0x0F || raw_buf[24] != 0x00)
    {
        return;
    }

    uint16_t tmp;

    tmp  = raw_buf[1] | ((uint16_t)raw_buf[2] << 8);
    sbus->ch[0]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[2] >> 3) | ((uint16_t)raw_buf[3] << 5);
    sbus->ch[1]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[3] >> 6) | ((uint16_t)raw_buf[4] << 2) | ((uint16_t)raw_buf[5] << 10);
    sbus->ch[2]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[5] >> 1) | ((uint16_t)raw_buf[6] << 7);
    sbus->ch[3]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[6] >> 4) | ((uint16_t)raw_buf[7] << 4);
    sbus->ch[4]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[7] >> 7) | ((uint16_t)raw_buf[8] << 1) | ((uint16_t)raw_buf[9] << 9);
    sbus->ch[5]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[9] >> 2) | ((uint16_t)raw_buf[10] << 6);
    sbus->ch[6]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[10] >> 5) | ((uint16_t)raw_buf[11] << 3);
    sbus->ch[7]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = raw_buf[12] | ((uint16_t)raw_buf[13] << 8);
    sbus->ch[8]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[13] >> 3) | ((uint16_t)raw_buf[14] << 5);
    sbus->ch[9]  = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[14] >> 6) | ((uint16_t)raw_buf[15] << 2) | ((uint16_t)raw_buf[16] << 10);
    sbus->ch[10] = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[16] >> 1) | ((uint16_t)raw_buf[17] << 7);
    sbus->ch[11] = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[17] >> 4) | ((uint16_t)raw_buf[18] << 4);
    sbus->ch[12] = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[18] >> 7) | ((uint16_t)raw_buf[19] << 1) | ((uint16_t)raw_buf[20] << 9);
    sbus->ch[13] = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[20] >> 2) | ((uint16_t)raw_buf[21] << 6);
    sbus->ch[14] = (int16_t)(tmp & 0x07FF) - 1024;

    tmp  = (raw_buf[21] >> 5) | ((uint16_t)raw_buf[22] << 3);
    sbus->ch[15] = (int16_t)(tmp & 0x07FF) - 1024;

    sbus->joy[0] = sbus->ch[0];
    sbus->joy[1] = sbus->ch[1];
    sbus->joy[2] = sbus->ch[2];
    sbus->joy[3] = sbus->ch[3];

    sbus->btn[0] = map_btn(sbus->ch[4]);
    sbus->btn[1] = map_btn(sbus->ch[5]);
    sbus->btn[2] = map_btn(sbus->ch[6]);
    sbus->btn[3] = map_btn(sbus->ch[7]);

    sbus->ch17  = (raw_buf[23] >> 0) & 0x01;
    sbus->ch18  = (raw_buf[23] >> 1) & 0x01;
    sbus->frame_lost= (raw_buf[23] >> 2) & 0x01;
    sbus->failsafe  = (raw_buf[23] >> 3) & 0x01;
}
/**
  * @brief 扩展串口事件回调，DMA+IDLE事件入口
  * @param huart 串口句柄
  * @param Size 本次收到的字节数量
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 判断是不是SBUS串口，并且收到完整25字节
    if(huart == sbus_data.sbus_huart && Size == SBUS_LEN)
    {
        SBUS_Decode(sbus_raw_buf, &sbus_data);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, sbus_raw_buf, SBUS_LEN);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

// x:输入[-1024,1024]
// dead_zone:输入侧死区阈值（正数，自己指定，例：30）
// out_max:输出最大值，输出范围[-out_max, out_max]
float map_sym_dead(int x, int dead_zone, float out_max)
{
    if(x > dead_zone)
    {
        return ( (float)(x - dead_zone) / (1024.0f - dead_zone) ) * out_max;
    }
    else if(x < -dead_zone)
    {
        return ( (float)(x + dead_zone) / (1024.0f - dead_zone) ) * out_max;
    }
    else
    {
        return 0.0f;
    }
}

int8_t map_btn(int16_t btn)
{
    if(btn < -500 && btn > -1024)
    {
        return -1;
    }
    else if(btn >-100 && btn < 100)
    {
        return 0;
    }
    else if(btn > 500 && btn < 1024)
    {
        return 1;
    }
    else
    {
        return -2;
    }
}

