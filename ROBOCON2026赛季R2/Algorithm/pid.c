#include "Pid.h"

#define NOW 1
#define LAST 0

pid_t pid_DJI_v1, pid_DJI_v2, pid_DJI_v3, pid_DJI_v4, pid_DJI_v5, pid_DJI_v6, pid_DJI_v7, pid_DJI_v8;
pid_t pid_DJI_p1, pid_DJI_p2, pid_DJI_p3, pid_DJI_p4, pid_DJI_p5, pid_DJI_p6, pid_DJI_p7, pid_DJI_p8;

pid_t pid_Follow_1, pid_Follow_2, pid_Follow_3, pid_Follow_4;
pid_t pid_Turn_1, pid_Turn_2, pid_Turn_3, pid_Turn_4;
pid_t pid_Auto_1, pid_Auto_2, pid_Auto_3, pid_Auto_4;

void abs_limit(float *a, float MAX)
{
  if (*a > MAX)
    *a = MAX;
  if (*a < -MAX)
    *a = -MAX;
}

void pid_reset(pid_t *pid)
{
  pid->pout = 0;
  pid->iout = 0;
  pid->out = 0;
  pid->dout = 0;
  pid->err[0] = 0;
  pid->err[1] = 0;
}
// ��ʼ��pid
void Pid_Init(pid_t *pid, float kp, float ki, float kd, float iout_max, float out_max, float dead_zone, float kf)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kf = kf;
  pid->iout_max = iout_max;
  pid->out_max = out_max;
  pid->dead_zone = dead_zone;
  pid_reset(pid);
}
float PID_Calc(pid_t *pid, float set, float get)
{
  pid->set = set;
  pid->get = get;
  pid->err[NOW] = set - get;

  if (fabs(pid->err[NOW]) <= pid->dead_zone)
  {
    pid->err[NOW] = 0;
  }

  // 常规PID计算
  pid->pout = pid->kp * pid->err[NOW];
  pid->iout += pid->ki * pid->err[NOW];
  pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]);

  abs_limit(&pid->iout, pid->iout_max);

  // 计算PID总输出
  pid->out = pid->pout + pid->iout + pid->dout;

  // 添加前馈项
  pid->out += pid->kf;

  abs_limit(&pid->out, pid->out_max);

  pid->err[LAST] = pid->err[NOW];

  return pid->out;
}
