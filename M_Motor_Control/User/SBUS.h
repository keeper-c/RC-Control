#ifndef __SBUS_H
#define __SBUS_H
#include "stm32f4xx_hal.h"

#define SBUS_LEN 25

typedef struct
{
    UART_HandleTypeDef *sbus_huart;
    int16_t ch[16];    // CH1‑CH16 0~2047
    uint8_t ch17;
    uint8_t ch18;
    int16_t joy[4];
    int8_t btn[4];
    uint8_t frame_lost;
    uint8_t failsafe;

}SBUS_t;

extern SBUS_t sbus_data;
extern uint8_t sbus_raw_buf[SBUS_LEN];

void SBUS_Init(SBUS_t *sbus, UART_HandleTypeDef *huart);
void SBUS_Decode(uint8_t *raw_buf, SBUS_t *sbus);
float map_sym_dead(int x, int dead_zone, float out_max);
int8_t map_btn(int16_t btn);


#endif

