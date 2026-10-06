#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Target;
    float Current;
    float Integral;
    float PrevError;
    float IntegralLimit;
    float OutputLimit;
    float Output;
    float dt;
} Struct_PID;

void PID_Init(Struct_PID *pid, float Kp, float Ki, float Kd, float Kf, float IntegralLimit, float OutputLimit, float dt);
float PID_Calculate(Struct_PID *pid);
void PID_SetTarget(Struct_PID *pid, float target);
void PID_SetCurrent(Struct_PID *pid, float current);
void PID_Reset(Struct_PID *pid);

#endif
