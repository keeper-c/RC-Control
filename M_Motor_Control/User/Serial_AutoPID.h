#ifndef __SERIAL_AUTOPID_H__
#define __SERIAL_AUTOPID_H__

#include "main.h"
#include "usart.h"
#include "drv_DJ_Control.h"
#include "string.h"
#define UART_RX_BUF_SIZE 64

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t  Frame_Handle;
    uint8_t  rx_buf[UART_RX_BUF_SIZE];
    uint8_t  pid_ready;
    float    Kp, Ki, Kd;
    uint8_t  target_ready;
    float    target;
} Serial_AutoPID_t;

extern Serial_AutoPID_t g_AutoPID;

void Serial_AutoPID_Init(Serial_AutoPID_t *obj, UART_HandleTypeDef *huart, uint8_t Handle_Handle);
void Serial_AutoPID_Send(Serial_AutoPID_t *obj, DJ_Motor_t *motor);

#endif
