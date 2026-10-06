#ifndef __ROB_POWER_H
#define __ROB_POWER_H

// ===================================================================
// 分层：rob_power — 动力接口层（rob_core 三层体系的中间层）
//   定义底盘和上层机构的动力系统函数指针表（ChsPower_t / ActPower_t），
//   将 power_bsp 硬件驱动抽象为统一接口，供 rob_core 层注入。
//   三层关系：rob_behave（行为基类）→ rob_power（动力接口）→ rob_core（对象组合）
//   【设计意图】函数指针表 = C 语言中的虚表（vtable），
//   上层模块不直接调用 power_bsp，而是通过 power->XXX() 间接调用，
//   实现硬件驱动的可替换性（仿真/实物切换只需换表内容）。
// ===================================================================

#include "main.h"

// 底盘动力接口 — 函数指针表，类似 C++ 虚表
typedef struct ChsPower
{
    // Wheel — 四轮麦轮速度控制（麦克纳姆轮解算后的各轮速度）
    //   spd1~spd4: 四个轮子的目标速度（rpm）
    //   coeff:     速度缩放系数（0.0=停转, 1.0=全速）
    void (*Wheel)(float spd1, float spd2, float spd3, float spd4, float coeff);

    // Track — 双履带位置+速度控制
    //   frontPos/frontSpd: 前履带位置（mm）/ 速度（rpm）
    //   rearPos/rearSpd:   后履带位置（mm）/ 速度（rpm）
    void (*Track)(float frontPos, float frontSpd, float rearPos, float rearSpd);
} ChsPower_t;

// 上层机构动力接口 — 龙门架各轴控制
typedef struct ActPower
{
    // Gantry_lift — 龙门架升降（垂直 Z 轴）
    //   pos: 目标位置（mm），spd: 运动速度（mm/s 或 rpm）
    void (*Gantry_lift)(float pos, float spd);

    // Gantry_push — 龙门架推杆（水平伸缩）
    //   pos: 目标位置（mm），spd: 运动速度
    void (*Gantry_push)(float pos, float spd);

    // Gantry_grip — 龙门架夹取（旋转角度控制）
    //   pos: 目标角度（度），spd: 旋转速度
    void (*Gantry_grip)(float pos, float spd);

    // Grip_arm — 夹爪臂旋转（独立于龙门架的辅助轴）
    //   pos: 目标角度（度），spd: 旋转速度
    void (*Grip_arm)   (float pos, float spd);
} ActPower_t;

// 动力接口实例 — 由 rob_power.c 定义，rob_core.c 注入到 RC_R2
extern ChsPower_t s_chs_power_cfg;
extern ActPower_t s_act_power_cfg;

#endif
