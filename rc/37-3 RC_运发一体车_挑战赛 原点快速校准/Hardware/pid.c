#include "pid.h"
#include <math.h>
void PID_Init(PID_ *pid, float Kp, float Ki, float Kd,
              float output_limit, float integral_limit, float dead_zone)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->dead_zone = dead_zone;
    PID_Reset(pid);
}

float PID_Calc(PID_ *pid, float set, float get)
{
    float error = set - get;

    if (fabs(error) <= pid->dead_zone)
    {
        error = 0;
    }
    float kp_out = pid->Kp * error;

    pid->integral += error;
    pid->integral = fmaxf(fminf(pid->integral, pid->integral_limit), -pid->integral_limit);
    float ki_out = pid->Ki * pid->integral;

    float kd_out = error - pid->last_error;
    kd_out = pid->Kd * kd_out;

    float output = kp_out + ki_out + kd_out;

    output = fmaxf(fminf(output, pid->output_limit), -pid->output_limit);

    pid->last_error = error;

    return output;
}

void PID_Reset(PID_ *pid)
{
    pid->integral = 0;
    pid->last_error = 0;
}
