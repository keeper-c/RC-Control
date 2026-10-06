//
// Created by keeper on 2026/7/26.
//

#include "MyDbus.h"

static uint16_t MapToDBUS(uint8_t val, uint8_t max_val)
{
    return (uint16_t)val * (DBUS_MAX_VAL - DBUS_MIN_VAL) / max_val + DBUS_MIN_VAL;
}

static uint16_t MapKeyToDBUS(uint8_t key)
{
    return key ? DBUS_MAX_VAL : DBUS_MIN_VAL;
}

void DBUS_SendFrame(Rx_Frame_t *rx)
{
    uint8_t frame[DBUS_FRAME_LEN];
    uint16_t ch0, ch1, ch2, ch3, s1, s2;

    ch0 = MapToDBUS(rx->Slider_Value[0], 100);
    ch1 = MapToDBUS(rx->Slider_Value[1], 100);
    ch2 = MapKeyToDBUS(rx->Key_State[0]);
    ch3 = MapKeyToDBUS(rx->Key_State[1]);
    s1  = MapKeyToDBUS(rx->Key_State[2]);
    s2  = MapToDBUS(rx->Battery_Value, 100);

    frame[0] = ch0 & 0xFF;
    frame[1] = ((ch1 & 0x1F) << 3) | ((ch0 >> 8) & 0x07);
    frame[2] = ((ch2 & 0x03) << 6) | ((ch1 >> 5) & 0x3F);
    frame[3] = ((ch3 & 0x01) << 7) | ((ch2 >> 2) & 0x7F);
    frame[4] = (ch3 >> 1) & 0xFF;
    frame[5] = ((s1 & 0x3F) << 2) | ((ch3 >> 9) & 0x03);
    frame[6] = ((s2 & 0x03) << 6) | ((s1 >> 6) & 0x3F);
    frame[7] = (s2 >> 2) & 0xFF;
    frame[8] = (s2 >> 10) & 0x01;

    for (uint8_t i = 9; i < DBUS_FRAME_LEN; i++)
    {
        frame[i] = 0;
    }

    HAL_UART_Transmit(&huart2, frame, DBUS_FRAME_LEN, 10);
}
