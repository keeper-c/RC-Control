/**
 ******************************************************************************
 * @file    rob_core.c
 * @brief   机器人核心封装层 — 三层 C 仿 OOP 体系顶层
 *
 * ── 三层架构 ──
 *
 *   rob_behave (行为基类)    纯类型 + 状态表，不依赖硬件
 *        ↓ 定义 ChsState_t / Action_t / Task_t / task_pkg[][]
 *   rob_power  (动力接口)    函数指针表 = C 语言的虚表，硬件驱动可替换
 *        ↓ 定义 ChsPower_t / ActPower_t，绑定到 power_bsp
 *   rob_core   (对象组合)    装配 RobotChs_t / RobotAct_t / Robot_t，对外暴露 RC_R2
 *        ↓  #include "rob_core.h" 即可操作完整机器人
 *   外部模块（rob_brain / rob_act / rob_chs / CommandTask）
 *
 * ── 设计要点 ──
 *
 *   [1] 函数指针表 = C 的虚表
 *       ChsPower_t / ActPower_t 将 power_bsp 驱动包装为统一接口。
 *       上层不直接调硬件函数，改调 power->Wheel(…) / power->Gantry_grip(…)。
 *       换硬件（仿真/实物、不同电机型号）只需换表内容，上层零改动。
 *
 *   [2] 二维动作表 = 工厂模式
 *       task_pkg[动作类型][任务阶段] — 6 种动作 × 4 个阶段（Idle/Init/Ready/Running）。
 *       加新动作 = 在 rob_behave.c 往表里加一行 + 在 rob_act.c 实现三个函数，
 *       brain/执行层的调度代码完全不用动。
 *
 *   [3] 决策-执行分离 + 脑拿锁
 *       brain 只写 RC_R2.Chs.now / Act.now → 执行层检测 now != last 自动切状态。
 *       brain 用 osMutexAcquire(mutex, 0) 非阻塞拿锁，拿不到就跳过本 tick；
 *       执行层在 Running 阶段持锁，防止 brain 打断正在跑的动作。
 *
 *   [4] 底盘决策优先级链
 *       后赋值覆盖前赋值：Idle → ApproachKFS → AutoTrack → Climb → Combine
 *       → StepUpDown → Stop。Stop 最后赋值，自然成为最高优先级。
 *
 *   [5] now / last 状态机
 *       RC_R2 的底盘和动作各持 now（当前）和 last（上一帧）。
 *       执行层每 tick 对比，now != last 时自动调 exit(旧) + entry(新)。
 *       entry 只跑一次，task 每 tick 循环跑。
 *
 *   [6] 子状态机
 *       每个动作内部还有 Idle→Init→Ready→Running 的阶段流转。
 *       brain 写 task_state，执行层按 task[state].execute() 索引执行，
 *       完成后 brain 切回 Idle 释放锁。
 *
 ******************************************************************************
 */

/*
 * ============================================================================
 *                       _ooOoo_
 *                      o8888888o
 *                      88" . "88
 *                      (| -_- |)
 *                      O\  =  /O
 *                   ____/`---'\____
 *                 .'  \\|     |//  `.
 *                /  \\|||  :  |||//  \
 *               /  _||||| -:- |||||-  \
 *               |   | \\\  -  /// |   |
 *               | \_|  ''\---/''  |   |
 *               \  .-\__  `-`  ___/-. /
 *             ___`. .'  /--.--\  `. . __
 *          ."" '<  `.___\_<|>_/___.'  >'"".
 *         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *         \  \ `-.   \_ __\ /__ _/   .-` /  /
 *    ======`-.____`-.___\_____/___.-`____.-'======
 *                       `=---='
 *    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *              佛祖保佑          永无BUG
 *                  ROBOCON 2026 必胜!
 * ============================================================================
 */

#include "rob_core.h"

/* ========== 底盘派生类实例 — 绑定状态表 & 动力配置 ========== */
// s_robot_chs 把底层定义的 ChsState 状态表（s_chs_state_pkg）
// 和 ChsPower 动力配置（s_chs_power_cfg）组装为一个完整的底盘对象，
// 在 RC_R2_Init() 中注入给 RC_R2.Chs。
static RobotChs_t s_robot_chs = {
    .name  = "little.R",
    .pkg   = s_chs_state_pkg,
    .power = &s_chs_power_cfg,
};

/* ========== 上层机构派生类实例 — 绑定动作表 & 动力配置 ========== */
// s_robot_act 把底层定义的 ActEvent 动作表（s_action_pkg）
// 和 ActPower 动力配置（s_act_power_cfg）组装为一个完整的动作对象，
// 在 RC_R2_Init() 中注入给 RC_R2.Act。
static RobotAct_t s_robot_act = {
    .name  = "little.C",
    .pkg   = s_action_pkg,
    .power = &s_act_power_cfg,
};

/* ==================== 全局唯一机器人实例 ==================== */
// 命名 "2026RC-R2"，eye 初始化为 0（机器人系统状态：kInit）。
// .Chs 和 .Act 由 RC_R2_Init() 注入，其余字段编译期归零。
// 外部模块通过 extern RC_R2 直接访问该实例的所有成员。
Robot_t RC_R2 = {
    .name = "2026RC-R2",
    .eye  = 0,
};

/* ========================= 初始化 ========================= */
// RC_R2_Init() — 系统启动时唯一初始化入口，必须在 osKernelStart() 前调用。
// 1. 将底盘和上层机构的 now/last 状态都初始化为 Idle，
//    确保状态机从已知安全状态开始运行。
// 2. 将底盘和上层机构派生对象注入全局实例 RC_R2，
//    之后所有模块通过 RC_R2.Chs.* / RC_R2.Act.* 即可操作完整机器人。
void RC_R2_Init(void)
{
    s_robot_chs.now  = s_robot_chs.pkg[kChsState_Idle];
    s_robot_chs.last = s_robot_chs.pkg[kChsState_Idle];

    s_robot_act.now  = s_robot_act.pkg[kActEvent_Idle];
    s_robot_act.last = s_robot_act.pkg[kActEvent_Idle];

    RC_R2.Chs = s_robot_chs;
    RC_R2.Act = s_robot_act;

    //RC_R2.Act.power->Gantry_lift(50.0f, 5.0f); // [调试] 初始化时抬起龙门架
}

/* ======================= 动作拷贝 ======================== */
// RobotAct_copy() — 将动作模板从动作表拷贝到当前运行时状态。
// 用于 brain 决策层切换动作时，同步 Rt->Act.now 的 id / task_state / task。
// @param Rt    目标机器人指针（通常为 &RC_R2）
// @param Act   目标动作枚举值，索引到 Rt->Act.pkg[] 动作表
// @param state 任务状态（kIdle/kInit/kReady/kRunning），设置 task_state
// @note  拷贝后，执行层（Action_Function）在下一轮 tick 检测到
//        now != last，自动触发 s_state_just_entered 标记。
void RobotAct_copy(Robot_t *Rt, ActEvent_e Act, int8_t state)
{   
    if (Rt == NULL || Rt->Act.pkg == NULL) return;
    Rt->Act.now.id         = Rt->Act.pkg[Act].id;
    Rt->Act.now.task_state = state;
    Rt->Act.now.task       = Rt->Act.pkg[Act].task;
}



