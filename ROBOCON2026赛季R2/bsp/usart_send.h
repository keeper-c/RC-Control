#ifndef UART_H
#define UART_H

#include "main.h"
#include "usart.h"
#include "pid.h"

#define COMM_DATA 51

// �������ھ������main.c�г�ʼ�������ⲿ���ã�
extern UART_HandleTypeDef huart6;

// ���͵��ֽ�
void uart_send_byte(uint8_t data);

// �����ַ���
void uart_send_string(const char *str);

extern uint8_t Rx_Data[COMM_DATA];
void Coordinate_Upper(uint8_t *data);
void UART6_Receive_Start(void);
extern uint32_t watch_dog_tick;

void UART6_DMA_Restart(void);

#endif
