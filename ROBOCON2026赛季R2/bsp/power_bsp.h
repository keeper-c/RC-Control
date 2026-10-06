#ifndef __POWER_BSP_H
#define __POWER_BSP_H

#include "main.h"
#include "can_bsp.h"
#include "pid.h"

void Wheel_config(float spd1, float spd2, float spd3, float spd4, float coeff);
void Track_config(float frontPos, float frontSpd, float rearPos, float rearSpd);

void Gantry_lift_config(float pos, float spd);
void Gantry_push_config(float pos, float spd);
void Gantry_grip_config(float pos, float spd);
void Grip_arm_config(float pos, float spd);


#endif
