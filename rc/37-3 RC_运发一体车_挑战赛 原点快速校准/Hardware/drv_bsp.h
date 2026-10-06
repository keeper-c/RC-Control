#ifndef __DRV_BSP_H
#define __DRV_BSP_H
#include "stdint.h"
float deg2rad(float deg);
uint8_t solve_angle_speed(float distance, float *out_angle_deg, float *out_speed);
void driver_ddd(float vx, float vy, float vw, float speed[4]);
float Forced_fitting(float x);
#endif
