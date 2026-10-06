#ifndef __ROB_CORE_H
#define __ROB_CORE_H

// ===================================================================
// 分层：rob_core — 核心封装层（rob_core 三层体系的顶层）
//   三层设计：rob_behave（行为基类）+  rob_power（动力接口）→ rob_core（对象组合）
//   本层组合前两层，构建派生类 RobotChs_t / RobotAct_t 和顶层 Robot_t，
//   对外暴露唯一机器人实例 RC_R2。
//   外部模块只需 #include "rob_core.h" 即可获得完整类型系统。
// ===================================================================

#include "main.h"
#include "rob_behave.h"
#include "rob_power.h"

// 派生类-机器人底盘（仅跨模块访问字段；模块私有状态为 rob_chs.c 内 static）
typedef struct RobotChs
{
    char *name;          // 底盘名称（调试用）
    ChsState_t now;      // 当前底盘状态（执行层每 tick 执行 now.Task）
    ChsState_t last;     // 上一 tick 底盘状态（用于检测状态切换，触发 entry/exit）
    ChsState_t *pkg;     // 底盘状态表指针（指向 s_chs_state_pkg 数组首元素）
    ChsPower_t *power;   // 底盘动力接口指针（指向 s_chs_power_cfg）
} RobotChs_t;

// 派生类-机器人上层机构（仅跨模块访问字段；模块私有状态为 rob_act.c 内 static）
typedef struct RobotAct
{
    char *name;          // 上层机构名称（调试用）
    Action_t now;        // 当前动作（执行层每 tick 执行 now.task[state].execute）
    Action_t last;       // 上一 tick 动作（用于检测动作/状态切换）
    Action_t *pkg;       // 动作表指针（指向 s_action_pkg 数组首元素）
    ActPower_t *power;   // 上层机构动力接口指针（指向 s_act_power_cfg）
} RobotAct_t;

// 机器人封装 — 将底盘和上层机构组合为完整机器人对象
typedef struct Robot
{
    char *name;          // 机器人名称（"2026RC-R2"）
    int16_t eye;          // 机器人系统状态码（kInit/kReady/kRunning 等）
    RobotChs_t Chs;      // 底盘 — 运动子系统
    RobotAct_t Act;      // 上机构 — 动作子系统

} Robot_t;

// 统一的接口暴露（全局唯一机器人实例）
extern Robot_t RC_R2;
void RC_R2_Init(void);
void RobotAct_copy(Robot_t *Rt, ActEvent_e Act, int8_t state);

// 机器人事件位（用于任务通知，StateFlag任务统一处理）
// brain 决策层用 xTaskNotify 发送这些位，唤醒 StateFlag_Function 执行对应逻辑
#define kRobEvt_Chassis  0x01  // 底盘状态机触发 — 决策底盘下一个目标状态
#define kRobEvt_Action   0x02  // 动作状态机触发 — 决策上层机构下一个动作
#define kRobEvt_Safety   0x04  // 动作完成安全等待触发 — 串口看门狗保护

// 任务状态枚举（Action_t.task_state / RobotAct_copy 的 state 参数）
#define kIdle            0      // 空闲 — 无任务运行
#define kInit            1      // 初始化 — 进入动作的一次性准备
#define kReady           2      // 准备 — 等待条件满足后进入运行
#define kRunning         3      // 运行 — 动作执行中，持锁防打断

#endif
