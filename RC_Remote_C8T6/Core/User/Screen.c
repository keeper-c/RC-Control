//
// Created by keeper on 2026/7/25.
//

#include "Screen.h"

Struct_Screen_t Screen = {0};


Component_Map_t Component_Map[COMPONENT_COUNT] = {
    {1, "key1.val",    &Screen.Key_State[0]},    // 前
    {2, "key2.val",    &Screen.Key_State[1]},    // 左
    {3, "key3.val",    &Screen.Key_State[2]},    // 右
    {4, "key4.val",    &Screen.Key_State[3]},    // 后
    {6, "slider1.val", &Screen.Slider_Value[0]},  // 左旋转
    {7, "slider2.val", &Screen.Slider_Value[1]},  // 右旋转
    {9, "switch1.val", &Screen.Switch_Value},     // 开关
};

void Screen_SendInt(char *MyTxt, uint8_t Value)
{
    char message[100] = "";
    snprintf(message, sizeof(message), "%s=%d\xff\xff\xff", MyTxt, Value);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

void Screen_SendString(char *MyTxt, char *ValueStr)
{
    char message[100] = "";
    snprintf(message, sizeof(message), "%s=\"%s\"\xff\xff\xff", MyTxt, ValueStr);
    HAL_UART_Transmit(&huart2, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

void Screen_SetValue(uint8_t ID, uint8_t Value)
{
    uint8_t i;
    for (i = 0; i < COMPONENT_COUNT; i++)
    {
        if (Component_Map[i].ID == ID)
        {
            *(uint8_t *)Component_Map[i].ValuePtr = Value;
            return;
        }
    }
}

void Screen_SendByName(char *Name)
{
    uint8_t i;
    for (i = 0; i < COMPONENT_COUNT; i++)
    {
        if (strcmp(Component_Map[i].Name, Name) == 0)
        {
            uint8_t value = *(uint8_t *)Component_Map[i].ValuePtr;
            Screen_SendInt(Component_Map[i].Name, value);
            return;
        }
    }
}

void Screen_SendFrame(void)
{
    if (Screen.Switch_Value == 0)
    {
        return;
    }

    uint8_t frame[TX_FRAME_LEN];

    frame[0] = TX_FRAME_HEADER;
    frame[1] = Screen.Key_State[0];
    frame[2] = Screen.Key_State[1];
    frame[3] = Screen.Key_State[2];
    frame[4] = Screen.Key_State[3];
    frame[5] = Screen.Slider_Value[0];
    frame[6] = Screen.Slider_Value[1];
    frame[7] = TX_FRAME_TAIL;

    HAL_UART_Transmit(&huart2, frame, TX_FRAME_LEN, HAL_MAX_DELAY);
}