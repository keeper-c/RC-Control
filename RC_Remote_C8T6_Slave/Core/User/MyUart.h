//
// Created by keeper on 2026/7/25.
//

#ifndef RC_REMOTE_C8T6_MYUART_H
#define RC_REMOTE_C8T6_MYUART_H

#include "main.h"
#include "usart.h"

#define RX_FRAME_LEN      8
#define RX_FRAME_HEADER   0xBB
#define RX_FRAME_TAIL     0xFF

#define RX_BUFFER_SIZE    128

typedef struct
{
    uint8_t Key_State[3];
    uint8_t Slider_Value[2];
    uint8_t Battery_Value;
    uint8_t Rx_Flag;
} Rx_Frame_t;

extern Rx_Frame_t Rx_Frame;

void MyUart_Init(void);

#endif //RC_REMOTE_C8T6_MYUART_H