/**
 ******************************************************************************
 * @file    rob_behave.c
 * @brief   行为基类层数据定义 — 底盘状态表和动作任务表
 * @details 定义底盘状态模板表（s_chs_state_pkg）和上层动作模板表（s_action_pkg），
 *          每个表项携带 entry/exit/task 函数指针，由 rob_core 层装配到机器人对象。
 *          新增动作/新增底盘状态只需在对应数组追加表项即可，无需改动上层代码。
 * @date    2026/5/9
 * @version V2.3
 ******************************************************************************
 */

#include "rob_behave.h"
#include "rob_act.h"
#include "rob_chs.h"

/* ========== 基类-机器人底盘状态拆解 ========== */
// s_chs_state_pkg[] — 底盘状态模板表，按 ChsState_e 枚举顺序排列。
// 每个表项包含：
//   id     — 状态 ID（= 数组下标，等于 ChsState_e 枚举值）
//   entry  — 状态进入回调（首次切换到此状态时调用，可 NULL）
//   exit   — 状态退出回调（切换到其他状态前调用，可 NULL）
//   Task   — 状态持续任务（每 tick 循环调用，可 NULL）
// 执行层（MOVE_Function）检测到 now.id != last.id 时自动调用 exit/entry。
ChsState_t s_chs_state_pkg[] = {
    {kChsState_Idle,        Chs_IdleEntry,        Chs_IdleExit,       Chs_IdleTask},
    {kChsState_StepUpDown,  Chs_StepUpDownEntry,  Chs_StepUpDownExit, Chs_StepUpDownTask},
    {kChsState_Climb,       Chs_ClimbEntry,       NULL,               Chs_ClimbTask},
    {kChsState_AutoTrack,   NULL,                 NULL,               Chs_AutoTrackTask},
    {kChsState_ApproachWp,  Chs_ApproachWpEntry,  NULL,               Chs_ApproachWpTask},
    {kChsState_ApproachKFS, Chs_ApproachKFSEntry, NULL,               Chs_ApproachKFSTask},
    {kChsState_Combine,     Chs_CombineEntry,     NULL,               Chs_CombineTask},
    {kChsState_Stop,        Chs_StopEntry,        NULL,               NULL},
};

/* ========== 动作函数注册库 ========== */
// task_pkg[][] — 二维任务表：[动作类型][任务状态]
//   第 0 维 = ActEvent_e 动作枚举（kActEvent_Idle / _Grab / _Assembly / …）
//   第 1 维 = 任务状态（0=kIdle, 1=kInit, 2=kReady, 3=kRunning）
//   每个槽位存储一个函数指针，执行层按 now.task_state 索引执行。
//   槽位为 NULL 表示该组合无操作（跳过执行）。
//
//   动作状态机流程：
//     Idle → Init（一次性初始化，如 GantryGetInit）
//          → Ready（准备/等待条件，如 GantryGetRdy）
//          → Running（主执行，通常持 mutex_ActHandle 锁）
//          → 完成后 brain 切回 Idle → 释放锁
static Task_t task_pkg[][4] = {
    /*           kIdle   kInit              kReady           kRunning         */
    /* Idle      */ {NULL, NULL,              NULL,             NULL            },
    /* Grab      */ {NULL, NULL,              Act_GrabReady,    Act_GrabTask    },
    /* Assembly  */ {NULL, NULL,              NULL,             Act_AssemblyTask},
    /* GantryGet */ {NULL, Act_GantryGetInit, Act_GantryGetRdy, Act_GantryGetTask},
    /* GantryPut */ {NULL, NULL,              Act_GantryPutRdy, Act_GantryPutTask},
    /* KfsMove   */ {NULL, NULL,              Act_KfsMoveRdy,   Act_KfsMoveTask },
};

/* ========== 基类-机器人动作拆解 ========== */
// s_action_pkg[] — 上层动作模板表，按 ActEvent_e 枚举顺序排列。
// 每个表项包含：
//   id         — 动作 ID（= 数组下标）
//   task_state — 初始任务状态（通常为 kIdle）
//   task       — 指向 task_pkg 对应行的指针（4 个函数指针的数组）
// brain 决策层调用 RobotAct_copy() 时从该表取模板写入 RC_R2.Act.now。
Action_t s_action_pkg[] = {
    {kActEvent_Idle,      kIdle, task_pkg[kActEvent_Idle]     },
    {kActEvent_Grab,      kIdle, task_pkg[kActEvent_Grab]     },
    {kActEvent_Assembly,  kIdle, task_pkg[kActEvent_Assembly] },
    {kActEvent_GantryGet, kIdle, task_pkg[kActEvent_GantryGet]},
    {kActEvent_GantryPut, kIdle, task_pkg[kActEvent_GantryPut]},
    {kActEvent_KfsMove,   kIdle, task_pkg[kActEvent_KfsMove]  },
};
