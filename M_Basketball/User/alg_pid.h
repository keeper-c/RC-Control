#ifndef __ALG_PID_H__
#define __ALG_PID_H__

/**
 * @brief 通用 PID 控制器(位置式/增量式, 思路与算法以大疆 C620 官方 Demo 为准)
 *
 * @note 算法(DJI 官方实现):
 *         位置式: err = target - feedback
 *                 integral += Ki*err, 钳位 ±integral_max
 *                 out = Kp*err + integral + Kd*(err - err_last), 钳位 ±out_max
 *         增量式: Δu = Kp*Δerr + Ki*err + Kd*(err - 2*err_last + err_llast)
 *                 out = last_out + Δu, 钳位 ±out_max
 *       误差三拍缓存 err[LLAST/LAST/NOW] 支撑两种形式;
 *       max_err 误差保护(|err|超限输出0) 与 deadband 死区(|err|过小输出0),
 *       0 = 禁用, 官方默认语义; 与官方唯一区别: 误差历史每拍推进,
 *       避免保护/死区退出后微分项引用旧误差产生冲击。
 *       使用: PID_Init(一次) -> 循环{ PID_Set_Target; out = PID_Calculate(feedback); }
 */
typedef enum
{
    PID_POSITION = 0,   /**< 位置式PID(速度环用这个) */
    PID_DELTA    = 1    /**< 增量式PID */
} PID_Mode_t;

/* 三拍误差下标(DJI 命名习惯) */
enum { LLAST = 0, LAST = 1, NOW = 2 };

typedef struct
{
    /* ---- 参数: PID_Init 一次设定 ---- */
    PID_Mode_t mode;    /**< 位置式 / 增量式 */
    float Kp;           /**< 比例增益 */
    float Ki;           /**< 积分增益(固定周期时已折入 dt) */
    float Kd;           /**< 微分增益 */
    float integral_max; /**< 积分限幅(|integral| ≤ integral_max) */
    float out_max;      /**< 输出限幅(|out| ≤ out_max) */
    float max_err;      /**< 误差保护阈值, 0=禁用 */
    float deadband;     /**< 死区阈值, 0=禁用 */

    /* ---- 状态 ---- */
    float target;       /**< 目标值(Set_Target 给定) */
    float err[3];       /**< 误差缓存 [LLAST, LAST, NOW] */
    float integral;     /**< 积分累加项(位置式) */
    float last_out;     /**< 上一拍输出(增量式累加基值) */
    float out;          /**< 本拍输出(已限幅) */
} PID_t;

/**
 * @brief 初始化 PID(仅此一次, 运行中不再改参数)
 * @param pid          PID 对象指针
 * @param mode         PID_POSITION 位置式 / PID_DELTA 增量式
 * @param Kp           比例增益
 * @param Ki           积分增益(固定控制周期时已折入 dt)
 * @param Kd           微分增益
 * @param integral_max 积分限幅(建议远小于 out_max)
 * @param out_max      输出限幅(执行器满量程)
 * @note max_err / deadband 默认禁用(0), 需要时初始化后直接写结构体字段
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
