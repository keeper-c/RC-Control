#include "pid.h"

void PID_Init(Struct_PID *pid, float Kp, float Ki, float Kd, float Kf, float IntegralLimit, float OutputLimit, float dt)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    pid->Integral = 0.0f;
    pid->PrevError = 0.0f;
    pid->IntegralLimit = IntegralLimit;
    pid->OutputLimit = OutputLimit;
    pid->Output = 0.0f;
    pid->dt = dt;
}

float PID_Calculate(Struct_PID *pid)
{
    float error = pid->Target - pid->Current;

    pid->Integral += error * pid->dt;
    if (pid->Integral > pid->IntegralLimit)
    {
        pid->Integral = pid->IntegralLimit;
    }
    else if (pid->Integral < -pid->IntegralLimit)
    {
        pid->Integral = -pid->IntegralLimit;
    }

    float derivative = (error - pid->PrevError) / pid->dt;
    pid->PrevError = error;

    pid->Output = pid->Kf * pid->Target + pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;

    if (pid->Output > pid->OutputLimit)
    {
        pid->Output = pid->OutputLimit;
    }
    else if (pid->Output < -pid->OutputLimit)
    {
        pid->Output = -pid->OutputLimit;
    }

    return pid->Output;
}

void PID_SetTarget(Struct_PID *pid, float target)
{
    pid->Target = target;
}

void PID_SetCurrent(Struct_PID *pid, float current)
{
    pid->Current = current;
}

void PID_Reset(Struct_PID *pid)
{
    pid->Integral = 0.0f;
    pid->PrevError = 0.0f;
    pid->Output = 0.0f;
}
