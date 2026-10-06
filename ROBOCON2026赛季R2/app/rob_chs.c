/**
 ******************************************************************************
 * @file    rob_chs.c
 * @brief   底盘状态机实现（Chs_MoveFunction 任务）
 * @details 实现 Idle / StepUpDown / Climb / AutoTrack / ApproachWp / ApproachKFS
 *          / Combine / Stop 共 8 个状态的 entry / exit / task 函数，
 *          注册到 rob_behave.c 的 s_chs_state_pkg[] 表中。
 *
 *          【执行框架】
 *          MOVE_Function 每 12ms tick 一次：
 *          1. 根据 VisData.Speed 切换速度系数（全速+/全速/停止）
 *          2. 检测 now.id != last.id → 调 exit + entry
 *          3. 调 now.Task() 执行当前状态的持续任务
 *          4. xTaskNotify 通知 brain 决策层 kRobEvt_Chassis
 *
 *          【核心约束】
 *          - 上下台阶/合体期间 Mux_updown=0，锁定 brain 底盘决策
 *          - MOVE 任务优先级 32，高于 brain(24)，确保执行层实时性
 *          - 详见 memory: [[任务优先级架构]]
 ******************************************************************************
 */
#include "rob_chs.h"
#include "rob_act.h"
#include "main.h"

// ==================== 上下台阶时序常量（ms） ====================
#define kUpslopePhase1Ms 1200   // 上坡第一阶段时长（履带+麦轮同时驱动）
#define kDownslopePhase1Ms 1300 // 下坡第一阶段时长
#define kTransition1Ms 1550     // 过渡阶段1时长（当前未使用）
#define kTransition2Ms 1450     // 过渡阶段2时长（减速/归位）

// ==================== 上下台阶 Z 轴判定阈值 ====================
#define kZAxisThresholdHigh 180 // Z 轴正向阈值（上台阶判定，当前未使用）
#define kZAxisThresholdLow -180 // Z 轴负向阈值（下台阶判定，当前未使用）

// ==================== 基板前推距离 ====================
#define kKfsForwardDist 1030 // 梅林目标基板前推固定距离 (mm)

// ==================== 底盘速度系数档位 ====================
#define kSpeedCoeffHigh1 1.1f // 全速+抵达目标（Speed=0 时使用）
#define kSpeedCoeffHigh 1.0f  // 全速抵达目标（Speed=1 时使用）
#define kSpeedCoeffMid 0.7f   // 低速视觉自动识别（上下台阶/夹取对准时）
#define kSpeedCoeffLow 0.4f   // 低速梅花桩上高精度微调（Speed=2 时使用）

#define SpeedCoeff(VisData_Speed)              \
    ((VisData_Speed == 0)   ? kSpeedCoeffHigh1 \
     : (VisData_Speed == 1) ? kSpeedCoeffHigh  \
     : (VisData_Speed == 2) ? 0.0f             \
                            : 0.4f)

// ==================== 攀爬子阶段枚举 ====================
typedef enum
{
    kStepClimb_Up,   // 上坡中
    kStepClimb_Done, // 下坡完成
} StepClimbState_e;

typedef enum
{
    kStepClimb_Up_combine,   // 合体上坡中
    kStepClimb_Done_combine, // 合体下坡中
} StepCombineState_e;

// ==================== 模块静态变量 ====================

// 速度系数：根据 VisData.Speed 动态选择
static float s_speed_coeff = kSpeedCoeffHigh;
// 台阶攀爬阶段状态
static StepClimbState_e s_stepclimb_state;     // 当前攀爬子阶段
static StepCombineState_e s_stepCombine_state; // 当前合体子阶段
static bool s_updown_active = false;           // 上下台阶机构是否正在运行
static uint32_t s_step_phase_tick;             // 攀爬阶段起始 SysTick

// KFS 基板接近位置快照（进入 ApproachKFS 状态时锁定，防止过程中坐标漂移）
static int16_t s_kfs_x_last; // 快照时 KFS 的 X 坐标
static int16_t s_kfs_y_last; // 快照时 KFS 的 Y 坐标
static int16_t s_cur_x_last; // 快照时机器人自身的 X 坐标
static int16_t s_cur_y_last; // 快照时机器人自身的 Y 坐标
static int16_t target_y;     // 临时目标 Y（ApproachWp 使用）
static int16_t target_x;     // 临时目标 X（ApproachWp 使用）

/***************************************************************
 * 静态辅助函数声明
 ****************************************************************/
static void Set_UpDown_Active(bool val); // 设置上下台阶机构运行标志

/**
 * @brief 底盘状态机任务 — 统一调度逻辑执行和状态切换
 *
 * 每 12ms tick 执行一次：
 * 1. 根据 Speed 信号更新速度系数（全速/低速/停止）
 * 2. 检测 GetKFS_SecondarySig → 强制 re-entry（异常兜底）
 * 3. 状态切换检测 → 调用上一状态 exit + 新状态 entry
 * 4. 执行当前状态的持续 Task
 * 5. 通知 brain 决策层 kRobEvt_Chassis
 *
 * @note 优先级 32，高于 brain(24)，保证执行层不被打断。
 */
void MOVE_Function(void const *argument)
{
    osDelay(100);
    for (;;)
    {
        // ===== 速度系数选择 =====
        s_speed_coeff = SpeedCoeff(VisData.Speed);
        /************************底盘状态机执行层******************************/
        // KFS 二次夹取：强制触发当前状态 entry（不经过 exit/entry 切换流程）
        if (GetKFS_SecondarySig == 1)
        {
            RC_R2.Chs.now.entry(); // 直接调用当前状态 entry
            GetKFS_SecondarySig = 0;
        }

        // ===== 状态切换检测 =====
        if (RC_R2.Chs.now.id != RC_R2.Chs.last.id)
        {
            ChsState_t entering = RC_R2.Chs.now;

            // 1. 先执行上一状态的退出回调
            if (RC_R2.Chs.last.exit != NULL)
            {
                RC_R2.Chs.last.exit();
            }

            // 2. 再执行新状态的进入回调
            if (entering.entry != NULL)
            {
                entering.entry();
            }

            // 3. 同步 last → now（下次 tick 不再触发切换）
            RC_R2.Chs.last = entering;
        }

        // ===== 执行当前状态的持续任务 =====
        if (RC_R2.Chs.now.Task != NULL)
        {
            RC_R2.Chs.now.Task();
        }

        // [未启用] 上下台阶超时安全监测
        // 如果上下台阶超过 10s 未完成，强制标记完成

        // 通知 brain 决策层：底盘执行层已完成本轮 tick
        xTaskNotify(StateFlag_TaskHandle, kRobEvt_Chassis, eSetBits);
        vTaskDelay(pdMS_TO_TICKS(12)); // 12ms 控制周期
    }
}

/**
 * @brief 重置上下台阶机构 — Z 轴目标变化时由 brain 调用
 */
void Chs_UpDownSigReset(void)
{
    Set_UpDown_Active(false);
}

/* ===================================================================
 * 底盘状态实现 — Entry 回调
 * =================================================================== */

/**
 * @brief Idle 进入 — 选择速度系数 + 履带归位
 *
 * 速度选择逻辑：
 *   - 有 Z 轴变化 (200/400/600) 或自身在 200 且目标为 0 → kSpeedCoeffMid（慢速对准）
 *   - Kfs_Get_Sig 1/3（准备夹取）→ kSpeedCoeffMid
 *   - 其他 → kSpeedCoeffHigh（全速导航）
 */

void Chs_IdleEntry(void)
{
    if ((Coord.target.z == 200 || Coord.target.z == 400 ||
         Coord.target.z == 600) ||
        (Coord.self.z == 200 && Coord.target.z == 0))
    {
        s_speed_coeff = kSpeedCoeffMid;
    }
    else
        s_speed_coeff = kSpeedCoeffHigh;

    if (VisData.Kfs_Get_Sig == 1 || VisData.Kfs_Get_Sig == 3)
    {
        s_speed_coeff = kSpeedCoeffMid;
    }
    RC_R2.Chs.power->Track(0.0f, 8.0f, 0.0f, 8.0f); // 履带归位
}

/**
 * @brief 上下台阶进入 — 判定上坡/下坡方向，记录起始 tick
 *
 * 上坡 (tar_z > cur_z) → kStepClimb_Up
 * 下坡 (tar_z < cur_z) → kStepClimb_Done
 *
 * @note 进入时设 Mux_updown=0，锁定 brain 的底盘决策（防止切换打断台阶动作）。
 *       详见 memory: [[电调烧毁防护]] — 台阶动作不可被中断。
 */
void Chs_StepUpDownEntry(void)
{
    // 保护双下台阶动作的进行，不被外界打扰--视觉雷达太飘了
    // 烧了电调的教训
    Mux_updown = 0; // 锁定 brain 底盘决策

    if (VisData.Combine_Sig == 0)
    {
        int16_t cur_z_last = Coord.self.z;
        int16_t tar_z_last = Coord.target.z;
        if (tar_z_last > cur_z_last)
        {
            s_stepclimb_state = kStepClimb_Up; // 上坡
        }
        else if (tar_z_last < cur_z_last)
        {
            s_stepclimb_state = kStepClimb_Done; // 下坡
        }
    }
    s_step_phase_tick = osKernelGetTickCount();
}

/**
 * @brief 合体进入 — 判定合体上坡/下坡方向
 *
 * Combine_Sig=2 → 合体上坡
 * Combine_Sig=3 → 合体下坡
 */
void Chs_CombineEntry(void)
{
    Mux_updown = 0;
    if (VisData.Combine_Sig == 2)
    {
        s_stepCombine_state = kStepClimb_Up_combine;
    }
    else if (VisData.Combine_Sig == 3)
    {
        s_stepCombine_state = kStepClimb_Done_combine;
    }
    s_step_phase_tick = osKernelGetTickCount();
}

/**
 * @brief 爬升进入
 */
void Chs_ClimbEntry(void)
{
    // RC_R2.Chs.power->Track(80.0f, 5.0f, 80.0f, 5.0f);
}

/**
 * @brief 靠近武器点进入 — 锁定目标坐标快照
 *
 * 目标 Y = 自身 Y + 213mm（武器点固定偏移）
 * 目标 X = 自身 X（保持当前 X 不变）
 */
void Chs_ApproachWpEntry(void)
{
    target_y = Coord.self.y + 213;
    target_x = Coord.self.x;
}

/**
 * @brief 靠近基板进入 — 快照 KFS 坐标和自身坐标
 *
 * 进入时锁定 KFS 和自身坐标快照，避免过程中视觉更新导致目标漂移。
 * ApproachKFSTask 基于快照坐标推算前方目标点。
 */
void Chs_ApproachKFSEntry(void)
{
    s_kfs_x_last = Coord.kfs.x;
    s_kfs_y_last = Coord.kfs.y;
    s_cur_x_last = Coord.self.x;
    s_cur_y_last = Coord.self.y;
}

/**
 * @brief 停止进入 — 所有动力归零
 */
void Chs_StopEntry(void)
{
    RC_R2.Chs.power->Wheel(0, 0, 0, 0, 0);
    RC_R2.Chs.power->Track(0.0f, 8.0f, 0.0f, 8.0f);
}

/* ===================================================================
 * 底盘状态实现 — Task 回调（持续执行）
 * =================================================================== */

/**
 * @brief Idle 任务 — 自由导航到目标坐标
 *
 * 调用 Coordinate_calc_norm() 计算四轮麦轮速度，按当前速度系数缩放。
 * 梅林区域坐标范围 2800-7600。
 */

void Chs_IdleTask(void)
{   
	  if(Grab_done==1)
		{
		   yawFirst = 0;
       fellowFirst = 1;
		}
    // 分配优先级
    if (yawFirst == 1 && fellowFirst == 1)
    {
        yawFirst = 0;
        fellowFirst = 1;
    }

    // 机器人先旋转后平移
    if (yawFirst == 1)
    {
        Coord.inst.x = Coord.self.x;
        Coord.inst.y = Coord.self.y;
        Coord.inst.angle = Coord.target.angle;

        if (abs(Coord.target.angle) < 20)
        {
            yawFirst = 0;
            Coord.inst.x = Coord.target.x;
            Coord.inst.y = Coord.target.y;
            Coord.inst.angle = Coord.target.angle;
        }
    }
    if (yawFirst == 0)
    {
        // 机器人先平移后旋转
        if (fellowFirst == 1) //&&Grab_count==0
        {
            Coord.inst.x = Coord.target.x;
            Coord.inst.y = Coord.target.y;
            Coord.inst.angle = 0;

            if (abs(Coord.error.x) < 500 && abs(Coord.error.y) < 500)
            {
                fellowFirst = 0;
                Coord.inst.x = Coord.target.x;
                Coord.inst.y = Coord.target.y;
                Coord.inst.angle = Coord.target.angle;
            }
        }
        else
        {
            Coord.inst.x = Coord.target.x;
            Coord.inst.y = Coord.target.y;
            Coord.inst.angle = Coord.target.angle;
        }
    }

    float mot_spd[4]; // 四轮电机速度缓存（Coordinate_calc_norm 输出）
    float coeff;
    Coordinate_calc_norm(Coord.inst, mot_spd);
    coeff = s_speed_coeff;
    RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], coeff);
}

/**
 * @brief 上下台阶任务 — 履带+麦轮协作跨越台阶
 *
 * 上坡 (kStepClimb_Up) 时序：
 *   段1 (0~1300ms):     前轮抬起上台阶
 *   段2 (1300~1800ms):  履带归零 + 麦轮 ±7.0rpm — 后轮推进
 *                       X/Y 误差 < 300 时 Mux_updown=1（提前解锁）
 *   段3 (1800ms+):      Mux_updown=1（强制解锁，防止死锁）
 *
 * 下坡 (kStepClimb_Done) 时序：
 *   段1 (0~1300ms):      前轮先下
 *   段2 (1300~1450ms):  过渡减速
 *   段3 (1450~1750ms):  后轮跟上
 *                       X/Y 误差 < 300 时解锁
 *   段4 (1750ms+):      强制解锁
 */
void Chs_StepUpDownTask(void)
{
    uint32_t elapsed = osKernelGetTickCount() - s_step_phase_tick;

    switch (s_stepclimb_state)
    {

    case kStepClimb_Up:
        if (elapsed < 120)
        {
            RC_R2.Chs.power->Wheel(-8.7f, -8.7f, 8.7f, 8.7f, 1.4f);
        }
        else if (elapsed >= 120 && elapsed < 950)
        {
            RC_R2.Chs.power->Track(75.0f, 9.0f, 130.0f, 11.0f);
            RC_R2.Chs.power->Wheel(-9.3f, -9.3f, 9.3f, 9.3f, 1.4f);
        }
        else if (elapsed >= 950 && elapsed < 1450)
        {
            RC_R2.Chs.power->Wheel(-9.4f, -9.4f, 9.4f, 9.4f, 1.4f);
            // 提前解锁：X/Y 误差 < 300mm 说明接近完成
            if (abs(Coord.target.x - Coord.self.x) < 300 &&
                abs(Coord.target.y - Coord.self.y) < 300)
            {
                s_updown_active = true;
                Mux_updown = 1;
            }
        }
        else if (elapsed >= 1450)
        {
            s_updown_active = true;
            Mux_updown = 1;
            RC_R2.Chs.power->Track(0.0f, 8.0f, 0.0f, 11.0f);
            RC_R2.Chs.power->Wheel(-7.0f, -7.0f, 7.0f, 7.0f, 1.4f);
        }
        break;

    case kStepClimb_Done:
        if (elapsed < 800)
        {
            // 前轮先下：前履带长行程，后履带短行程
            RC_R2.Chs.power->Track(125.0f, 10.0f, 70.0f, 8.0f);
            RC_R2.Chs.power->Wheel(-5.8f, -5.8f, 5.8f, 5.8f, 1.4f);
        }
        else if (elapsed >= 800 && elapsed < 1390)
        {
            RC_R2.Chs.power->Wheel(-8.5f, -8.5f, 8.5f, 8.5f, 1.4f);
        }
        else if (elapsed >= 1390 && elapsed < 1760)
        {
            // 过渡减速
            RC_R2.Chs.power->Track(70.0f, 3.0f, 30.0f, 3.0f);
            RC_R2.Chs.power->Wheel(-4.3f, -4.3f, 4.3f, 4.3f, 1.4f);
        }
        else if (elapsed >= 1760 && elapsed < 2150)
        {
            RC_R2.Chs.power->Track(0.0f, 5.0f, 0.0f, 5.0f);
            RC_R2.Chs.power->Wheel(0, 0, 0, 0, 1.4f);
        }
        else if (elapsed >= 2150)
        {
            s_updown_active = true;
            Mux_updown = 1;
        }
        break;
    }
}

/**
 * @brief 合体任务 — 上下台阶 + 合体协调运动
 *
 * kStepClimb_Up_combine（合体上坡）：
 *   段1 (0~1300ms):   履带 75/138mm + 麦轮 ±10.4rpm（与上坡段1相同）
 *   段2 (1300~1800ms): 履带归零 + 麦轮 ±7.0rpm
 *   段3 (1800ms+):     g_combine_achieve=1, Mux_updown=1（合体完成）
 *
 * kStepClimb_Done_combine（合体下坡）：
 *   段1 (0~1300ms):   履带 70/136mm + 麦轮 ±7.6rpm
 *   段2 (1300~1450ms): 履带 70/30mm + 麦轮 ±7.0rpm
 *   段3 (1450~1850ms): 履带 30/30mm + 麦轮停转
 *   段4 (1850ms+):     合体完成
 */
void Chs_CombineTask(void)
{
    uint32_t elapsed = osKernelGetTickCount() - s_step_phase_tick;
    switch (s_stepCombine_state)
    {
    case kStepClimb_Up_combine:
        if (elapsed < 800)
        {
            RC_R2.Chs.power->Wheel(-2.4f, -2.4f, 2.4f, 2.4f, 1.4f);
        }
        else if (elapsed >= 800 && elapsed < kUpslopePhase1Ms + 2100)
        {
            RC_R2.Chs.power->Track(75.0f, 9.0f, 138.0f, 11.0f);
            RC_R2.Chs.power->Wheel(-4.4f, -4.4f, 4.4f, 4.4f, 1.4f);
        }
        else if (elapsed >= kUpslopePhase1Ms + 2100 && elapsed < kUpslopePhase1Ms + 2500)
        {
            RC_R2.Chs.power->Track(0.0f, 8.0f, 0.0f, 11.0f);
            RC_R2.Chs.power->Wheel(-4.0f, -4.0f, 4.0f, 4.0f, 1.4f);
        }
        else if (elapsed >= kUpslopePhase1Ms + 2500)
        {
            RC_R2.Chs.power->Wheel(0.0f, 0.0f, 0.0f, 0.0f, 1.4f);
            g_combine_achieve = 1; // 合体已完成 → brain 保持 Stop
            Mux_updown = 1;
        }
        break;
    
    case kStepClimb_Done_combine:
        if (elapsed < 500)
        {   
            // 前轮先下：前履带长行程，后履带短行程
            RC_R2.Chs.power->Track(70.0f, 8.0f, 125.0f, 10.0f);
        }   
        if (elapsed >= 500 && elapsed < 1800)
        {   
            RC_R2.Chs.power->Wheel(5.8f, 5.8f, -5.8f, -5.8f, 1.4f);
        }
        else if (elapsed >= 1800 && elapsed < 2000)
        {
            // 过渡减速
            RC_R2.Chs.power->Track(70.0f, 3.0f, 30.0f, 3.0f);
            RC_R2.Chs.power->Wheel(4.3f, 4.3f, -4.3f, -4.3f, 1.4f);
        }
        else if (elapsed >= 2000 && elapsed < 2400)
        {
            RC_R2.Chs.power->Track(0.0f, 5.0f, 0.0f, 5.0f);
            RC_R2.Chs.power->Wheel(0, 0, 0, 0, 1.4f);
        }
        else if (elapsed >= 2400)
        {
            Mux_updown = 1;
        }
        break;
    }
}

/**
 * @brief 爬升任务 — 朝目标方向全速前进
 */
void Chs_ClimbTask(void)
{
    float mot_spd[4];
    Coordinate_calc_norm(Coord.target, mot_spd);
    RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], kSpeedCoeffHigh);
}

/**
 * @brief 靠近武器点任务 — 直线接近射击位
 *
 * 使用 demo 坐标（Entry 中锁定的 target_x / target_y），
 * Coordinate_calc_norm 解算后全速驱动。
 */
void Chs_ApproachWpTask(void)
{
    Coord.demo.y = target_y;
    Coord.demo.x = target_x;
    Coord.demo.angle = 0;

    float mot_spd[4];
    Coordinate_calc_norm(Coord.demo, mot_spd);
    RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], kSpeedCoeffHigh);
}

/**
 * @brief 视觉自动对准任务 — Coordinate_calc_auto 自动跟踪 detect 点
 *
 * Wp_Trk_Sig=2 时用中速 (0.7)，其余用全速 (1.0)。
 */
void Chs_AutoTrackTask(void)
{
    float mot_spd[4];
    Coordinate_calc_auto(Coord.detect, mot_spd);
    if (VisData.Wp_Trk_Sig == 2)
    {
        RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], kSpeedCoeffMid);
    }
    else
    {
        RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], kSpeedCoeffHigh);
    }
}

/**
 * @brief 靠近基板任务 — 基于快照坐标推算 KFS 前方目标点并接近
 *
 * 逻辑：
 *   1. 计算快照时 KFS 与自身的相对位置 (x_err_last, y_err_last)
 *   2. 根据主方向（X 或 Y），在 KFS 后方 kKfsForwardDist 处设定目标点
 *   3. Z != 0 时: 按 X/Y 误差中较大的那个方向设定偏移
 *      Z == 0 时: 仅按 X 方向设定偏移（一层基板特殊情况）
 *   4. Coordinate_calc_norm 解算 + 全速驱动 + 履带归位
 *
 * @note 履带始终保持归位 (0, 8, 0, 8)，避免影响麦轮运动。
 */
void Chs_ApproachKFSTask(void)
{
    int16_t y_err_last;
    int16_t x_err_last;

    x_err_last = s_kfs_x_last - s_cur_x_last;
    y_err_last = s_kfs_y_last - s_cur_y_last;
    int16_t kfs_x = Coord.kfs.x;
    int16_t kfs_y = Coord.kfs.y;

    // Z != 0（多层基板）：按主方向偏移
    if (Coord.self.z != 0)
    {
        if (abs(y_err_last) > abs(x_err_last))
        {
            if (y_err_last > 0)
                kfs_y = s_kfs_y_last - kKfsForwardDist;
            else if (y_err_last < 0)
                kfs_y = s_kfs_y_last + kKfsForwardDist;
        }
        if (abs(y_err_last) < abs(x_err_last))
        {
            if (x_err_last > 0)
                kfs_x = s_kfs_x_last - kKfsForwardDist;
            else if (x_err_last < 0)
                kfs_x = s_kfs_x_last + kKfsForwardDist;
        }
    }

    // Z == 0（一层基板）：仅按 X 方向偏移
    if (Coord.self.z == 0)
    {
        if (x_err_last > 0)
            kfs_x = s_kfs_x_last - kKfsForwardDist;
        else if (x_err_last < 0)
            kfs_x = s_kfs_x_last + kKfsForwardDist;
    }

    if (yawFirst == 1)
    {
        Coord.inst.x = Coord.self.x;
        Coord.inst.y = Coord.self.y;
        if (abs(Coord.target.angle) < 40)
        {
            yawFirst = 0;
					  Coord.inst.x = kfs_x;
            Coord.inst.y = kfs_y;
        }
    }
    else
    {
        Coord.inst.x = kfs_x;
        Coord.inst.y = kfs_y;
    }
    Coord.inst.angle = Coord.target.angle; 
    float mot_spd[4];
    Coordinate_calc_norm(Coord.inst, mot_spd);
    RC_R2.Chs.power->Wheel(mot_spd[0], mot_spd[1], mot_spd[2], mot_spd[3], kSpeedCoeffHigh);
    RC_R2.Chs.power->Track(0.0f, 8.0f, 0.0f, 8.0f);
}

/* ===================================================================
 * 底盘状态实现 — Exit 回调
 * =================================================================== */

/**
 * @brief Idle 退出 — 停止所有轮子
 */
void Chs_IdleExit(void)
{
    RC_R2.Chs.power->Wheel(0, 0, 0, 0, 0);
}

/**
 * @brief 上下台阶退出 — 标记机构运行完成 + 防止切任务不触发释放
 */
void Chs_StepUpDownExit(void)
{
    s_updown_active = true; // 标记完成
}

/* ===================================================================
 * 外部安全获取接口
 * =================================================================== */

/**
 * @brief 设置上下台阶机构运行标志
 */
static void Set_UpDown_Active(bool val)
{
    s_updown_active = val;
}

/**
 * @brief 查询上下台阶机构是否正在运行
 * @return true=运行中（brain 不可打断台阶），false=空闲
 */
bool Chs_IsUpDownActive(void)
{
    return s_updown_active;
}
