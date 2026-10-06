//
// Created by keeper on 2026/7/18.
//

#ifndef STMF4_CAN_MOTORCONTROL_CALLBACK_H
#define STMF4_CAN_MOTORCONTROL_CALLBACK_H

#include "stm32f4xx_hal.h"
#include "drv_can.h"

void UART2_Callback(uint8_t *rx_buf, uint16_t length);

#endif //STMF4_CAN_MOTORCONTROL_CALLBACK_H