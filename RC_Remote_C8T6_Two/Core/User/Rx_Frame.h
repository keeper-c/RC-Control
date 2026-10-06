//
// Created by keeper on 2026/7/30.
//

#ifndef RC_REMOTE_C8T6_TWO_RX_FRAME_H
#define RC_REMOTE_C8T6_TWO_RX_FRAME_H

#include "main.h"
#include "usart.h"

#define RX_FRAME_LEN 6
#define RX_FRAME_HEADER 0xAA
#define RX_FRAME_TAIL 0xFF

#define RX_BUFFER_SIZE 20

typedef enum
{
    RX_STATE_IDLE = 0,
    RX_STATE_DATA = 1,
    RX_STATE_TAIL = 2,
}Rx_State;

typedef struct
{
    uint8_t Value[RX_FRAME_LEN];
    Rx_State State;
    uint8_t Index;
    uint8_t Flag;
}Rx_Frame;

extern Rx_Frame RFrame;


#endif //RC_REMOTE_C8T6_TWO_RX_FRAME_H