#ifndef __ROB_BEHAVE_H
#define __ROB_BEHAVE_H

// ===================================================================
// 分层：rob_behave — 行为基类层（rob_core 三层体系的最底层）
//   定义状态机的基础类型（状态/动作结构体、事件枚举、事件位掩码），
//   供 rob_core 层组合使用。
//   不依赖任何 rob_* 文件，不引入硬件，是纯类型定义层。
//   三层关系：rob_behave（行为基类）→ rob_power（动力接口）→ rob_core（对象组合）
//   设计意图：所有状态/动作都"注册"到 rob_behave 定义的数据表中，
//   上层通过表索引而非硬编码函数名调用，新增行为只需追加表项。
// ===================================================================

#include "main.h"

// 底盘状态结构体 — 状态机的一个节点
// 类比 UML 状态机的 State：entry = 进入动作，exit = 退出动作，Task = do/activity
typedef struct ChsState
{
    int8_t id;              // 状态 ID（等于 ChsState_e 枚举值）
    void (*entry)(void);    // 进入回调 — 首次切换到此状态时执行一次
    void (*exit)(void);     // 退出回调 — 离开此状态时执行一次
    void (*Task)(void);     // 持续任务 — 在该状态期间每 tick 循环执行
} ChsState_t;

// 任务函数指针包装 — 动作的最小执行单元
typedef struct Task
{
    void (*execute)(void);  // 指向具体动作实现函数
} Task_t;

// 动作结构体 — 携带子状态机的动作节点
// 每个动作内部有一个 task_state 子状态机（Idle→Init→Ready→Running），
// 使动作可以分阶段执行（准备→运行→完成），而非单次调用。
typedef struct Action
{
    int8_t id;              // 动作 ID（等于 ActEvent_e 枚举值）
    int8_t task_state;      // 任务子状态：0=kIdle, 1=kInit, 2=kReady, 3=kRunning
    Task_t *task;           // 指向 task_pkg 对应行的指针 — 4 个阶段的函数指针数组
} Action_t;

// 枚举动作事件 — 上层机构的完整动作集合
// 每个动作对应一个 ActEvent_e 值，同时也是 s_action_pkg[] 的索引
typedef enum
{
    kActEvent_Idle,         // 空闲 — 无动作，上层机构保持安全位置
    kActEvent_Grab,         // 抓取子弹头 — 视觉对准后抓取弹药
    kActEvent_Assembly,     // 装配 — 将子弹头装入发射机构
    kActEvent_GantryGet,    // 龙门架夹取KFS — 夹取关键功能组件
    kActEvent_GantryPut,    // 龙门架放置KFS — 将 KFS 放置到目标位置
    kActEvent_KfsMove,      // KFS移动 — 转移 KFS 位置（重新定位）
} ActEvent_e;

// 枚举底盘状态 — 底盘运动的所有状态
// 每个状态对应一个 ChsState_e 值，同时也是 s_chs_state_pkg[] 的索引
typedef enum
{
    kChsState_Idle,         // 空闲移动 — 自由运动到目标坐标
    kChsState_StepUpDown,   // 上下台阶 — 跨越台阶障碍物
    kChsState_Climb,        // 爬升 — 攀爬斜坡/垂直面
    kChsState_AutoTrack,    // 视觉自动对准 — 靠视觉传感器锁定目标
    kChsState_ApproachWp,   // 靠近武器点 — 接近射击位置
    kChsState_ApproachKFS,  // 靠近基板 — 接近 KFS 组件位置
    kChsState_Combine,      // 合体 — 两台机器人对接组合
    kChsState_Stop,         // 停止 — 底盘锁定，所有轮/履带停转
} ChsState_e;

// 状态表/动作表 — 由 rob_behave.c 定义，rob_core.c 注入 RC_R2
extern Action_t s_action_pkg[];
extern ChsState_t s_chs_state_pkg[];

#endif
