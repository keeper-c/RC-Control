#ifndef __PID_H
#define __PID_H
#include "main.h"
#include "can_bsp.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kf;

    float set;
    float get;
    float err[2];

    float pout;
    float iout;
    float dout;
    float out;

    float iout_max;
    float out_max;
    float dead_zone;

} pid_t;

extern pid_t pid_DJI_v1, pid_DJI_v2, pid_DJI_v3, pid_DJI_v4, pid_DJI_v5, pid_DJI_v6, pid_DJI_v7, pid_DJI_v8;
extern pid_t pid_DJI_p1, pid_DJI_p2, pid_DJI_p3, pid_DJI_p4, pid_DJI_p5, pid_DJI_p6, pid_DJI_p7, pid_DJI_p8;
extern pid_t pid_Follow_1, pid_Follow_2, pid_Follow_3, pid_Follow_4;
extern pid_t pid_Turn_1, pid_Turn_2, pid_Turn_3, pid_Turn_4;
extern pid_t pid_Auto_1, pid_Auto_2, pid_Auto_3, pid_Auto_4;

void Pid_Init(pid_t *pid, float kp, float ki, float kd, float iout_max, float out_max, float dead_zone, float kf);
void pid_reset(pid_t *pid);
float PID_Calc(pid_t *pid, float set, float get);

#endif
