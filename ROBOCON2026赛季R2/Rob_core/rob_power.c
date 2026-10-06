/**
 ******************************************************************************
 * @file    rob_power.c
 * @brief   动力接口层实现 — 绑定硬件驱动到函数指针表
 * @details 定义底盘和上层机构的动力系统函数指针表实例，
 *          将 power_bsp 提供的硬件驱动函数装配为统一接口。
 *          实例以 extern 形式供 rob_core 层注入到机器人对象。
 * @date    2026/5/9
 * @version V2.2
 ******************************************************************************
 */

#include "rob_power.h"
#include "power_bsp.h"

/* ========== 底盘动力接口实例 — 绑定麦轮 & 履带驱动 ========== */
// s_chs_power_cfg 将 power_bsp 层提供的 Wheel_config / Track_config
// 硬件驱动函数绑定到统一接口，供 RC_R2.Chs.power->Wheel(…) 调用。
// 参数说明见 ChsPower_t 定义（rob_power.h）。
ChsPower_t s_chs_power_cfg = {
    .Wheel = Wheel_config,   // 四轮麦轮速度控制
    .Track = Track_config,   // 双履带位置+速度控制
};

/* ========== 上层机构动力接口实例 — 绑定龙门架驱动 ========== */
// s_act_power_cfg 将 power_bsp 层提供的龙门架各轴驱动函数
// 绑定到统一接口，供 RC_R2.Act.power->Gantry_lift(…) 等调用。
ActPower_t s_act_power_cfg = {
    .Gantry_lift = Gantry_lift_config,   // 龙门架升降（Z轴直线）
    .Gantry_push = Gantry_push_config,   // 龙门架推杆（水平伸缩）
    .Gantry_grip = Gantry_grip_config,   // 龙门架夹取（旋转角度）
    .Grip_arm    = Grip_arm_config,      // 夹爪臂旋转
};
