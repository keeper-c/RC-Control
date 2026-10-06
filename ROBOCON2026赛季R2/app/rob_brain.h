#ifndef __ROB_BRAIN_H
#define __ROB_BRAIN_H

// ===================================================================
// 分层：rob_brain — 应用层-决策调度
//   机器人顶层决策状态机（StateFlag_Function），根据传感器/视觉数据
//   切换 RC_R2 的底盘状态和上层动作，是行为决策的最高层。
//   依赖：rob_chs.h / rob_act.h（底盘和动作的具体实现）。
//
//   【决策优先级】
//   1. 动作完成/保护 → Stop（最高优先，持锁期间不允许移动）
//   2. 上下台阶      → StepUpDown（Z 轴变化时，先于自由移动判断）
//   3. 视觉引导动作  → AutoTrack / ApproachKFS / Combine（按信号优先级）
//   4. 自由移动      → Idle（默认状态，全场导航）
// ===================================================================

#include "main.h"

#include "rob_core.h"

#include "navigation.h"
#include "power_bsp.h"

// 全局标志 — 由 rob_chs.c / rob_brain.c 协作维护
extern int8_t g_combine_achieve;      // 合体完成标志（1=已完成合体，底盘保持 Stop）
extern int8_t GetKFS_SecondarySig;    // KFS 二次夹取信号（1=需重新进入 KFS 接近）
extern int8_t Mux_updown;             // 上下台阶互斥锁（1=允许决策, 0=上下台阶中，锁定）

#endif
