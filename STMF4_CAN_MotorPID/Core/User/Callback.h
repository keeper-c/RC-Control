//
// Created by keeper on 2026/7/18.
//

#ifndef STMF4_CAN_MOTORCONTROL_CALLBACK_H
#define STMF4_CAN_MOTORCONTROL_CALLBACK_H

#include "stm32f4xx_hal.h"

void UART2_Callback(uint8_t *rx_buf, uint16_t length);

extern float torque;
#endif //STMF4_CAN_MOTORCONTROL_CALLBACK_H