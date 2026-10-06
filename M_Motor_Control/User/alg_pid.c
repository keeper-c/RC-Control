#include "alg_pid.h"
#include <math.h>

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
    pid->deadband = 0.0f;
    pid->sep      = 0.0f;   /* 积分分离默认关 */
    pid->alpha    = 1.0f;   /* 微分低通默认不滤波 */
    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->err[LLAST] = 0.0f;
    pid->err[LAST]  = 0.0f;
    pid->err[NOW]   = 0.0f;
    pid->target   = 0.0f;
    pid->integral = 0.0f;
    pid->out      = 0.0f;
    pid->d_filt   = 0.0f;
    pid->prev_fb  = 0.0f;
    pid->first    = 1u;
}

void PID_Set_Target(PID_t *pid, float target)
{
    pid->target = target;
}

/**
 * @brief PID 单拍运算(位置式, 在固定周期任务/中断中调用)
 */
float PID_Calculate(PID_t *pid, float feedback)
{
    /* 误差历史推进: err[NOW] = target - feedback
       (每拍执行, 区别于官方在保护分支提前返回, 避免历史失真) */
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST]  = pid->err[NOW];
    pid->err[NOW]   = pid->target - feedback;

    /* 死区(DJI): |err| 在死区内本拍输出 0, 防止稳态抖动 */
    if(pid->deadband != 0.0f &&
       (pid->err[NOW] < pid->deadband && pid->err[NOW] > -pid->deadband))
    {
        pid->prev_fb = feedback;   /* 同上: 死区内也要推进反馈基准 */
        pid->out = 0.0f;
        return pid->out;
    }

    /* 首拍保护: 只出 P, 防止 prev/历史初值导致 D/积分假尖峰 */
    if(pid->first)
    {
        pid->first  = 0u;
        pid->d_filt = 0.0f;
        pid->out = pid->Kp * pid->err[NOW];
        abs_limit(&pid->out, pid->out_max);
        return pid->out;
    }

    /* 梯形积分 + 积分分离(|err|>sep 不积分) + 钳位 */
    if(pid->sep <= 0.0f || fabsf(pid->err[NOW]) <= pid->sep)
    {
        pid->integral += pid->Ki * (pid->err[NOW] + pid->err[LAST]) * 0.5f;
        abs_limit(&pid->integral, pid->integral_max);
    }

    /* 微分先行: 对反馈微分(负号=阻尼), 目标阶跃时 D 不炸;
       dt=固定周期已折入 Kd; 低通滤量化噪声 */
    float d = -(feedback - pid->prev_fb);
    pid->d_filt = pid->alpha * d + (1.0f - pid->alpha) * pid->d_filt;

    /* 未限幅输出, 用于抗饱和判断 */
    float out_unsat = pid->Kp * pid->err[NOW]
                    + pid->integral
                    + pid->Kd * pid->d_filt;

    /* 钳位式抗积分饱和: 输出超限且误差同向 -> 回退本拍积分(冻结) */
    uint8_t sat_high = (out_unsat >  pid->out_max) && (pid->err[NOW] > 0.0f);
    uint8_t sat_low  = (out_unsat < -pid->out_max) && (pid->err[NOW] < 0.0f);
    if(sat_high || sat_low)
    {
        if(pid->sep <= 0.0f || fabsf(pid->err[NOW]) <= pid->sep)
            pid->integral -= pid->Ki * (pid->err[NOW] + pid->err[LAST]) * 0.5f;
        abs_limit(&pid->integral, pid->integral_max);
        out_unsat = pid->Kp * pid->err[NOW]
                  + pid->integral
                  + pid->Kd * pid->d_filt;
    }

    pid->out = out_unsat;
    abs_limit(&pid->out, pid->out_max);
    pid->prev_fb = feedback;

    return pid->out;
}
