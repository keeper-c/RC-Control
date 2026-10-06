#ifndef __L1S_MEASURE_H_
#define __L1S_MEASURE_H_

#include "stm32f4xx_hal.h"

void L1s_one_measure(UART_HandleTypeDef *huart);
void L1s_fast_measure(UART_HandleTypeDef *huart);
void L1s_continue_measure(UART_HandleTypeDef *huart);
void L1s_stop_measure(UART_HandleTypeDef *huart);
float L1s_distance_solving(uint8_t *ch);

#endif
