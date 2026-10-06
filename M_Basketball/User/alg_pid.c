#include "alg_pid.h"

/* 统一限幅(DJI): 绝对值钳位 */
static void abs_limit(float *a, float limit_max)
{
    if(*a >  limit_max) *a =  limit_max;
    if(*a < -limit_max) *a = -limit_max;
}

void PID_Init(PID_t *pid, PID_Mode_t mode, float Kp, float Ki, float Kd,
              float integral_max, float out_max)
{
    pid->mode = mode;
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral_max = integral_max;
    pid->out_max = out_max;
    pid->max_err  = 0.0f;   /* 默认禁用, 需要时初始化后直接写字段 */
    pid->deadband = 0.0f;
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->err[LLAST] = 0.0f;
    pid->err[LAST]  = 0.0f;
    pid->err[NOW]   = 0.0f;
    pid->target   = 0.0f;
    pid->integral = 0.0f;
    pid->last_out = 0.0f;
    pid->out      = 0.0f;
}

void PID_Set_Target(PID_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief PID 单拍运算(位置式/增量式由 mode 决定, 算法同 DJI 官方 Demo)
 */
float PID_Calculate(PID_t *pid, float feedback)
{
    /* 误差历史推进: err[NOW] = target - feedback
       (每拍执行, 区别于官方在保护分支提前返回, 避免历史失真) */
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->err[NOW]   = pid->target - feedback;

    /* 误差保护(DJI): |err| 超限认为异常(反馈丢失/失稳), 切断输出保安全 */
    if(pid->max_err != 0.0f &&
       (pid->err[NOW] > pid->max_err || pid->err[NOW] < -pid->max_err))
    {
        pid->out = 0.0f;
        return pid->out;
    }

    /* 死区(DJI): |err| 在死区内本拍输出 0, 防止稳态抖动 */
    if(pid->deadband != 0.0f &&
       (pid->err[NOW] < pid->deadband && pid->err[NOW] > -pid->deadband))
    {
        pid->out = 0.0f;
        return pid->out;
    }

    if(pid->mode == PID_POSITION)
    {
        /* 位置式: 积分累加后钳位(官方仅此抗饱和手段) */
        pid->integral += pid->Ki * pid->err[NOW];
        abs_limit(&pid->integral, pid->integral_max);

        pid->out = pid->Kp * pid->err[NOW]
                 + pid->integral
                 + pid->Kd * (pid->err[NOW] - pid->err[LAST]);
        abs_limit(&pid->out, pid->out_max);
    }
    else    /* PID_DELTA 增量式 */
    {
        /* 增量式: Δu = Kp*Δe + Ki*e + Kd*(e - 2*e_last + e_llast), 输出累加 */
        float delta_u = pid->Kp * (pid->err[NOW] - pid->err[LAST])
                      + pid->Ki * pid->err[NOW]
                      + pid->Kd * (pid->err[NOW] - 2.0f * pid->err[LAST] + pid->err[LLAST]);

        pid->out = pid->last_out + delta_u;
        abs_limit(&pid->out, pid->out_max);
    }

    pid->last_out = pid->out;
    return pid->out;
}
