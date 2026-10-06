#ifndef __ROB_ACT_H
#define __ROB_ACT_H

// ===================================================================
// 分层：rob_act — 应用层-上层动作
//   实现上层机构各动作的具体 task 函数，
//   运行时注册到 rob_core 层的 Action_t 动作表中执行。
//   依赖：rob_core.h（获取 RC_R2 实例和 Action_t 类型）。
// ===================================================================
#include "main.h"

#include "rob_core.h"

#include "rob_brain.h"
#include "can_bsp.h"
#include "navigation.h"
#include "CommandTask.h"
#include "power_bsp.h"

// ==================== 抓取动作 ====================
void Act_GrabReady(void);      // 抓取准备 — 龙门架就位 + 夹爪张开
void Act_GrabTask(void);       // 抓取执行 — 夹取 → 抬起 → 完成

// ==================== 装配动作 ====================
void Act_AssemblyTask(void);   // 装配执行 — 将弹药装入发射机构

// ==================== 龙门架夹取KFS动作 ====================
void Act_GantryGetInit(void);  // 夹取初始化 — 龙门架预就位
void Act_GantryGetRdy(void);   // 夹取准备 — 配置参数 + 移动到起始位
void Act_GantryGetTask(void);  // 夹取执行 — 多段时序控制夹取过程

// ==================== 龙门架放置KFS动作 ====================
void Act_GantryPutRdy(void);   // 放置准备 — 视觉扫描 + 龙门架就位
void Act_GantryPutTask(void);  // 放置执行 — 推杆伸出 → 释放 → 归位

// ==================== KFS 转移动作 ====================
void Act_KfsMoveRdy(void);     // KFS转移准备 — 读取信号判断移动模式
void Act_KfsMoveTask(void);    // KFS转移执行 — 按 VisData.Kfs_Get_Sig 分支执行

extern int8_t yawFirst;
extern int8_t Grab_count;
extern int8_t fellowFirst;
extern int8_t Grab_done;

#endif




