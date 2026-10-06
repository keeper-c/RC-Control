//
// Created by keeper on 2026/7/30.
//

#ifndef RC_REMOTE_C8T6_TWO_MY_SCRREN_H
#define RC_REMOTE_C8T6_TWO_MY_SCRREN_H

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEYNUM 6

#define TX_BUFFER_SIZE 8
#define RX_BUFFER_SIZE 128

#define RX_FRAME_LEN 6

#define RX_FRAME_HEADER 0xAA
#define RX_FRAME_TAIL 0xFF

#define TX_FRAME_HEADER 0xAA
#define TX_FRAME_TAIL 0xFF
#define TX_FRAME_LEN 8

typedef struct KeyMap_t
{
    uint8_t ID;
    void* Value;
} KeyMap_t;

typedef struct
{
    uint16_t Key_State[KEYNUM];
    uint8_t Rx_Flag;
} Scrren;

extern KeyMap_t KeyMap[KEYNUM];
extern Scrren Scrren1;

extern uint8_t Rx_Buffer[];
extern uint8_t Tx_Buffer[];

void Uart_Start(UART_HandleTypeDef *huart);
void SetValue(Scrren* scrren, uint8_t ID, uint16_t Value);
void SendFrame(Scrren* scrren);
#ifdef __cplusplus
}
#endif

#endif //RC_REMOTE_C8T6_TWO_MY_SCRREN_H