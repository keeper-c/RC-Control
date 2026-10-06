#ifndef __PID_H
#define __PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float last_error;

    float output_limit;   // 输出限幅值（绝对值）
    float integral_limit; // 积分限幅值（绝对值）
    float dead_zone;      // 死区阈值
} PID_;

// 初始化PID控制器
void PID_Init(PID_ *pid, float Kp, float Ki, float Kd, float output_limit, float integral_limit, float dead_zone);

// 计算PID输出
float PID_Calc(PID_ *pid, float set, float get);

// 重置控制器状态
void PID_Reset(PID_ *pid);

#endif
