#ifndef __ROB_CHS_H
#define __ROB_CHS_H

// ===================================================================
// 分层：rob_chs — 应用层-底盘
//   实现底盘状态机各状态的具体 entry/exit/task 函数，
//   运行时注册到 rob_core 层的 ChsState_t 状态表中执行。
//   依赖：rob_core.h（获取 RC_R2 实例和 ChsState_t 类型）。
// ===================================================================

#include "main.h"

#include "rob_core.h"
#include "rob_brain.h"
#include "navigation.h"
#include "CommandTask.h"
#include "power_bsp.h"

// ==================== 外部安全接口 ====================

// Chs_UpDownSigReset — 重置上下台阶机构状态（目标 Z 变更时由 brain 调用）
void Chs_UpDownSigReset(void);

// Chs_IsUpDownActive — 查询上下台阶机构是否正在运行
// @return true=运行中（brain 不可打断），false=空闲
bool Chs_IsUpDownActive(void);

// ==================== 状态持续任务（Task 回调） ====================
void Chs_IdleTask(void);           // 空闲 — 自由导航到目标坐标
void Chs_StepUpDownTask(void);     // 上下台阶 — 履带+麦轮协作跨越台阶
void Chs_ClimbTask(void);          // 爬升 — 朝目标方向全速前进
void Chs_AutoTrackTask(void);      // 视觉对准 — Coordinate_calc_auto 自动跟踪
void Chs_ApproachWpTask(void);     // 靠近武器点 — 直线接近射击位
void Chs_ApproachKFSTask(void);    // 靠近基板 — 推算 KFS 前方位置并接近
void Chs_CombineTask(void);        // 合体 — 上下台阶+合体协调运动

// ==================== 状态进入回调（Entry 回调） ====================
void Chs_IdleEntry(void);          // Idle 进入 — 选址速度系数 + 履带归位
void Chs_StepUpDownEntry(void);    // 上下台阶进入 — 判定上坡/下坡方向
void Chs_ClimbEntry(void);         // 爬升进入
void Chs_ApproachWpEntry(void);    // 靠近武器点进入 — 锁定目标坐标快照
void Chs_ApproachKFSEntry(void);   // 靠近基板进入 — 快照 KFS 和自身坐标
void Chs_CombineEntry(void);       // 合体进入 — 判定合体上坡/下坡方向
void Chs_StopEntry(void);          // 停止进入 — 所有动力归零

// ==================== 状态退出回调（Exit 回调） ====================
void Chs_IdleExit(void);           // Idle 退出 — 停止所有轮子
void Chs_StepUpDownExit(void);     // 上下台阶退出 — 标记机构运行完成


#endif
