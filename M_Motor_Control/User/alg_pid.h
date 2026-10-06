#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"

/**
 * @brief 通用 PID 控制器(位置式)
 *
 * @note 位置式算法(参考梯形规划+级联位置环项目四件套内核):
 *         梯形积分:   integral += Ki*(err+err_last)/2 (dt 已折入 Ki)
 *         积分分离:   |err| > sep 时本拍不积分(sep<=0 禁用)
 *         积分限幅:   ±integral_max 钳位
 *         抗饱和:     未限幅输出超限且误差同向 -> 冻结本拍积分
 *         微分先行:   d = -(fb - fb_last)/dt 对反馈微分, 目标阶跃时 D 不炸
 *         微分低通:   d_filt = alpha*d + (1-alpha)*d_filt, 抑制量化噪声
 *         首拍保护:   首拍只出 P, 防历史初值假尖峰
 *         out = Kp*err + integral + Kd*d_filt, 钳位 ±out_max
 *       误差三拍缓存 err[LLAST/LAST/NOW]; deadband 死区保留(0=禁用)。
 *       PID_Init 签名不变, sep/alpha 默认关闭, 需要时初始化后直接写结构体字段。
 *       使用: PID_Init(一次) -> 循环{ PID_Set_Target; out = PID_Calculate(feedback); }
 */
typedef enum
{
    PID_POSITION = 0    /**< 位置式PID(速度环/位置环都用这个) */
} PID_Mode_t;

/* 三拍误差下标(DJI 命名习惯) */
enum { LLAST = 0, LAST = 1, NOW = 2 };

typedef struct
{
    /* ---- 参数: PID_Init 一次设定 ---- */
    PID_Mode_t mode;    /**< 控制器形式(当前仅 PID_POSITION) */
    float Kp;           /**< 比例增益 */
    float Ki;           /**< 积分增益(固定周期时已折入 dt) */
    float Kd;           /**< 微分增益 */
    float integral_max; /**< 积分限幅(|integral| ≤ integral_max) */
    float out_max;      /**< 输出限幅(|out| ≤ out_max) */
    float deadband;     /**< 死区阈值, 0=禁用 */
    float sep;          /**< 积分分离窗口: |err|>sep 本拍不积分, <=0=禁用 */
    float alpha;        /**< 微分低通系数 0~1: 1=不滤波(默认), 越小越平滑 */

    /* ---- 状态 ---- */
    float target;       /**< 目标值(Set_Target 给定) */
    float err[3];       /**< 误差缓存 [LLAST, LAST, NOW] */
    float integral;     /**< 积分累加项(位置式) */
    float out;          /**< 本拍输出(已限幅) */
    float d_filt;       /**< 微分低通滤波状态(位置式) */
    float prev_fb;      /**< 上一拍反馈(微分先行用) */
    uint8_t first;      /**< 首拍标志(防历史初值假尖峰) */
} PID_t;

/**
 * @brief 初始化 PID(仅此一次, 运行中不再改参数)
 * @param pid          PID 对象指针
 * @param mode         控制器形式(当前仅 PID_POSITION)
 * @param Kp           比例增益
 * @param Ki           积分增益(固定控制周期时已折入 dt)
 * @param Kd           微分增益
 * @param integral_max 积分限幅(建议远小于 out_max)
 * @param out_max      输出限幅(执行器满量程)
 * @note deadband 默认禁用(0), 需要时初始化后直接写结构体字段
 */
void PID_Init(PID_t *pid, PID_Mode_t mode, float Kp, float Ki, float Kd,
              float integral_max, float out_max);

/**
 * @brief 设置目标值(每拍调用)
 * @param target 目标值(与反馈同量纲)
 */
void PID_Set_Target(PID_t *pid, float target);

/**
 * @brief PID 单拍运算(在固定周期任务/中断中调用)
 * @param feedback 当前反馈值
 * @return 本拍输出(已限幅, 与 out 字段相同)
 */
float PID_Calculate(PID_t *pid, float feedback);

/**
 * @brief 复位运行状态(清误差历史/积分/输出, 不清参数)
 * @note 切换模式或重新使能前调用, 防止旧积分冲输出
 */
void PID_Reset(PID_t *pid);

#endif
