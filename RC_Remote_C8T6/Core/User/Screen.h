//
// Created by keeper on 2026/7/25.
//

#ifndef RC_REMOTE_C8T6_SCREEN_H
#define RC_REMOTE_C8T6_SCREEN_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

#define COMPONENT_COUNT 7

#define TX_FRAME_LEN      8
#define TX_FRAME_HEADER   0xAA
#define TX_FRAME_TAIL     0xFF

typedef struct
{
    uint8_t  ID;
    char    *Name;
    void    *ValuePtr;
} Component_Map_t;

typedef struct
{
    uint8_t Key_State[4];
    uint8_t Slider_Value[2];
    uint8_t Switch_Value;
} Struct_Screen_t;

extern Struct_Screen_t Screen;
extern Component_Map_t Component_Map[COMPONENT_COUNT];

void Screen_SendInt(char *MyTxt, uint8_t Value);
void Screen_SendString(char *MyTxt, char *ValueStr);
void Screen_SetValue(uint8_t ID, uint8_t Value);
void Screen_SendByName(char *Name);
void Screen_SendFrame(void);

#endif //RC_REMOTE_C8T6_SCREEN_H