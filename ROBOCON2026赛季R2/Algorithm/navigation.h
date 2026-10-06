#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"
#include "usart_send.h"
#include "CommandTask.h"
#include "pid.h"

void Coordinate_calc_norm(Coordinate target, float vout[4]);
void Coordinate_calc_auto(Coordinate target, float vout[4]);

void offset_inst_get(Coordinate robot_self, Coordinate *inst);

#endif
