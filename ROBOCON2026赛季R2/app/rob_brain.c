/**
 ******************************************************************************
 * @file    rob_brain.c
 * @brief   机器人顶层决策调度（StateFlag_Function 任务）
 * @details 根据视觉/坐标数据判断底盘目标状态和上层目标动作，直接改写
 *          RC_R2.Chs.now / RC_R2.Act.now，由执行层任务响应状态切换。
 *
 *          【三层决策框架】
 *          1. kRobEvt_Chassis  — 底盘运动状态决策（优先级从低到高）
 *          2. kRobEvt_Action   — 上层机构动作决策（信号驱动）
 *          3. kRobEvt_Safety   — 安全等待层（串口看门狗，当前未启用）
 *
 *          【底盘决策优先级】（高→低，后赋值覆盖前赋值）
 *            Stop > StepUpDown > Combine > Climb > AutoTrack > ApproachKFS > Idle
 *
 *          【动作决策】脑拿锁决策模式：
 *          brain 先 osMutexAcquire(mutex_ActHandle)，
 *          拿到锁 → 写入 Chs.now/Act.now → 释放锁。
 *          执行层在 Running 阶段持锁防止 brain 打断。
 *          详见 memory: [[动作完成自动放锁]], [[互斥锁保护状态机]]
 ******************************************************************************
 */
#include "rob_brain.h"
#include "rob_chs.h"
#include "main.h"
#include "CommandTask.h"

// ==================== 上下台阶距离阈值 ====================
int16_t kUpDownLnogDist=1300; // 远距离判断阈值（mm）— X/Y 误差在此范围内允许上台阶
#define kUpDownShortDist 300    // 近距离阈值（mm，当前未使用）

// ==================== 通信消息 ====================
static const uint8_t kMsgArrive[8] = "arrive\r\n"; // 到达目标通知上位机
static const char kMsgInit[32]  = "RC_R2初始化成功\r\n"; // 打印日志
static const char kMsgAlarm[32] = "警报串口中断意外关闭\r\n"; // 打印日志

// ==================== 全局状态标志 ====================
int8_t g_combine_achieve = 0;   // 合体完成标志（1=已完成）
int8_t GetKFS_SecondarySig = 0; // KFS 二次夹取信号（1=需重新进入 KFS 接近态）
int8_t Mux_updown = 1;          // 上下台阶互斥开关（1=允许决策，0=锁定/正在执行）

/***************************************************************
 * 静态辅助函数声明
 ****************************************************************/
static bool Judge_UpDown(void);         // 判断是否需要上下台阶
static void chassis_TargetFilter(void); // 底盘到达检测 + 发送 "arrive" 通知
static void Action_start_should(void);  // 动作启动条件判断（对准精度 + 计数消抖）

/**
 * @brief 状态标志任务 — 机器人行为模式的顶层决策调度
 *
 * 任务由 Action_Function（kRobEvt_Action）和 MOVE_Function（kRobEvt_Chassis）
 * 通过 xTaskNotify 唤醒，每次唤醒只处理对应事件位。
 *
 * ulNotificationValue 位含义：
 *   bit0 (kRobEvt_Chassis) — 底盘运动状态决策
 *   bit1 (kRobEvt_Action)  — 上层动作决策
 *   bit2 (kRobEvt_Safety)  — 安全等待层（#if 0 禁用）
 */
uint32_t s_step_phase_tick;
void StateFlag_Function(void const *argument)
{   
	  //打印日志-串口是否成功，任务是否崩掉
	  uart_send_string(kMsgInit);
    uint32_t ulNotificationValue = 0;
    static Coordinate target_last = {0}; // 上一次目标坐标，用于检测 Z 轴变化触发 UpDownSigReset
   
#if 0  // 仿真视觉初始化调试
    //RC_R2.Act.power->Gantry_lift(160.0f, 5.0f);
		VisData.Wp_Trk_Sig=1;
    s_step_phase_tick = osKernelGetTickCount();
#endif
    
    for (;;)
    {   
        // 阻塞等待任意事件通知（Chassis / Action / Safety）
        ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        // Z 轴变化 → 重置上下台阶机构（目标高度改变需重新判断上下台阶）
        if (target_last.z != Coord.target.z)
        {
            Chs_UpDownSigReset();
            target_last = Coord.target;
            Mux_updown = 1;
        }

#if 0 // 动态调试
				
				if(osKernelGetTickCount()-s_step_phase_tick>7000)
				{
				   VisData.Wp_Trk_Sig=3;
				}
#endif 
       
        /*************** 机器人底盘运动状态机决策层 （0位为1才执行）*****************/
        if (ulNotificationValue & kRobEvt_Chassis)
        {   
            // 上下台阶互斥检查：机构运行时 Mux_updown=0，不执行底盘决策
            if (Mux_updown == 1)
            {   
                /*通过赋值顺序实现底盘状态优先级仲裁*/
                // 默认状态：空闲移动
                RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_Idle];

                // ---- 上下台阶判断 ----
                // 条件：目标 Z 在 200/400/600 或 (自身在 200 且目标在 0)
                //      且目标角度误差 < 10°，且上下台阶机构未在运行
                if (Chs_IsUpDownActive() == false)
                {
                    if (((Coord.target.z == 200 || Coord.target.z == 400 ||
                          Coord.target.z == 600) ||
                         (Coord.self.z == 200 && Coord.target.z == 0)) &&
                        abs(Coord.target.angle) < 10)
                    {
                        if (Judge_UpDown())
                            RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_StepUpDown];
                    }
                }

                // ---- 合体状态 ----
                if (VisData.Combine_Sig == 2 || VisData.Combine_Sig == 3)
                {
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_Combine];
                    if (VisData.Combine_Sig == 3)
                        g_combine_achieve = 0; // 合体下台阶：重置完成标志
                }

                // ---- 爬升 ----
                if (VisData.Climb_Sig == 1)
                {
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_Climb];
                }

                // ---- 视觉自动对准（多信号触发） ----
                // Combine_Sig=1 合体对准 / Wp_Trk_Sig=2 武器对准
                // Kfs_Get_Sig=5 夹取失败重试 / 3 平地夹取 / 8 备用
                if (VisData.Combine_Sig == 1 || VisData.Wp_Trk_Sig == 2 ||
                    VisData.Kfs_Get_Sig == 5 || VisData.Kfs_Get_Sig == 3 ||
                    VisData.Kfs_Get_Sig == 8)
                {
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_AutoTrack];
                }

                // ---- 靠近基板（KFS 接近） ----
                if (VisData.Kfs_Get_Sig == 2 || VisData.Kfs_Get_Sig == 10 || VisData.Kfs_Get_Sig == 7)
                {
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_ApproachKFS];
                }

                // ---- 停止（最高优先级，覆盖前面的赋值） ----
                // 条件：动作正在执行 / 合体已完成 / KFS 特殊信号
                if (RC_R2.Act.now.task_state == kRunning ||
                    g_combine_achieve == 1 || VisData.Kfs_Get_Sig == 9 ||
                    VisData.Kfs_Get_Sig == 6)
                {
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_Stop];
                }

                // ---- 串口看门狗断连进行机器人保护 TIM1 ----
                if (RobotProtect == 1)
                {
					          //串口中断断连或意外FreeRTOS关闭中断
					          uart_send_string(kMsgAlarm); 
                    RC_R2.Chs.now = RC_R2.Chs.pkg[kChsState_Stop]; 
                }

                // 底盘到达检测（发送 "arrive" 给上位机）
                chassis_TargetFilter();
            }
        }

        /****************** 机器人特定动作决策层（1位为1才执行）*****************/
        if (ulNotificationValue & kRobEvt_Action)
        {
            int8_t state = RC_R2.Act.now.task_state;

            // 脑拿锁：拿到 mutex_ActHandle 才能决策动作
            // 如果执行层正在持锁（Running 阶段），brain 会跳过本 tick
            if (osMutexAcquire(mutex_ActHandle, 0) == osOK)
            {
                // ---- 无视觉信号 → 空闲 ----
                if (!(VisData.Asm_Trk_Sig || VisData.Kfs_Get_Sig ||
                      VisData.Wp_Trk_Sig || VisData.Kfs_Put_Sig))
                {
                    RobotAct_copy(&RC_R2, kActEvent_Idle, kIdle);
                }

                // ---- 抓取武器头 ----
                if (VisData.Wp_Trk_Sig != 0)
                {
                    // Sig=1/2: 对准中 → 准备
                    if (VisData.Wp_Trk_Sig == 1 || VisData.Wp_Trk_Sig == 2)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_Grab, kReady);
                    }
                    // Sig=3 且未在运行: 就绪 → 准备（由 Action_start_should 升级为 Running）
                    if (VisData.Wp_Trk_Sig == 3 && state != kRunning)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_Grab, kReady);
                    }
                }

                // ---- 装配 ----
                if (VisData.Asm_Trk_Sig == 1)
                {
                    RobotAct_copy(&RC_R2, kActEvent_Assembly, kRunning);
                }

                // ---- 龙门架夹取秘籍（KFS） ----
                if (VisData.Kfs_Get_Sig != 0 && VisData.Kfs_Get_Sig != 8)
                {
                    static int8_t Kfs_Get_Sig_last;
                    if (Kfs_Get_Sig_last != VisData.Kfs_Get_Sig)
                    {
                        if (Kfs_Get_Sig_last == 6 && VisData.Kfs_Get_Sig == 7)
                        {
                            RobotAct_copy(&RC_R2, kActEvent_KfsMove, kReady);
                        }
                        Kfs_Get_Sig_last = VisData.Kfs_Get_Sig;
                    }
                    if (VisData.Kfs_Get_Sig == 1)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kInit); // 初始化
                    }
                    else if (VisData.Kfs_Get_Sig == 2)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kReady); // 准备
                    }
                    else if (VisData.Kfs_Get_Sig == 8)
                    {
                        // 雷达自瞄对齐
                    }
                    else if (VisData.Kfs_Get_Sig == 9)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kRunning);
                    }
                    // 夹取失败，重新夹取（保持 Ready，由 Action_start_should 升级）
                    else if (VisData.Kfs_Get_Sig == 5)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kRunning);
                    }
                    // 夹取平地上的 KFS
                    else if (VisData.Kfs_Get_Sig == 3)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kReady);
                    }
                    else if (VisData.Kfs_Get_Sig == 4)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_GantryGet, kRunning);
                    }
                    else if (VisData.Kfs_Get_Sig == 10 || (VisData.Kfs_Get_Sig == 7 && state != kRunning))
                    {
                        RobotAct_copy(&RC_R2, kActEvent_KfsMove, kReady);
                    }
                    // 转移 KFS 位置
                    else if (VisData.Kfs_Get_Sig == 6)
                    {
                        RobotAct_copy(&RC_R2, kActEvent_KfsMove, kRunning);
                    }
                }

                // ---- 龙门架放置秘籍 ----
                if (VisData.Kfs_Put_Sig == 1)
                {
                    RobotAct_copy(&RC_R2, kActEvent_GantryPut, kReady);
                }
                if (VisData.Kfs_Put_Sig == 2)
                {
                    RobotAct_copy(&RC_R2, kActEvent_GantryPut, kRunning);
                }

                // ---- 动作启动条件判断（对准精度 + 计数消抖） ----
                Action_start_should();

                osMutexRelease(mutex_ActHandle); // 放锁 → 执行层可继续
            }
        }

        /****************** 机器人动作完成安全等待层 （2位为1才执行）*****************/
#if 0 
        if (ulNotificationValue & kRobEvt_Safety)
        {
        }
#endif
    }
}

/**
 * @brief 判断是否需要上下台阶
 *
 * 逻辑：
 *   1. 比较 X/Y 误差的绝对值，按较大方向判断
 *   2. 根据 Z 轴变化方向调整距离阈值：
 *      - 目标 Z >= 自身 Z（上坡）→ kUpDownLnogDist = 1400mm
 *      - 目标 Z <  自身 Z（下坡）→ kUpDownLnogDist = 1280mm
 *   3. 如果 X/Y 误差在阈值范围内 → 需要上台阶
 *
 * @return true=需要上下台阶, false=距离太远，自由移动靠近
 */
static bool Judge_UpDown(void)
{
    int16_t x_error = Coord.target.x - Coord.self.x;
    int16_t y_error = Coord.target.y - Coord.self.y;

    if (abs(x_error) > abs(y_error))
    {
        // 根据 Z 方向动态调整阈值
        if (Coord.target.z - Coord.self.z >= 0)
        {
            if (Coord.self.z == 0)
            {
                kUpDownLnogDist = 1400; // 上坡：容错更大
            }
            else
                kUpDownLnogDist = 1500;
        }
        else
        {
            kUpDownLnogDist = 1300; // 下坡：稍保守1280
        }

        // X 方向判断
        if (x_error > 0)
        {
            if (x_error < kUpDownLnogDist)
                return true;
        }
        else if (x_error < 0)
        {
            if (x_error > -kUpDownLnogDist)
                return true;
        }
    }

    // Y 方向判断（Y 误差为主方向时）
    if (abs(x_error) < abs(y_error))
    {
        if (y_error > 0)
        {
            if (y_error < kUpDownLnogDist)
                return true;
        }
        else if (y_error < 0)
        {
            if (y_error > -kUpDownLnogDist)
                return true;
        }
    }
    return false;
}

/**
 * @brief 底盘到达检测 — 位置误差收敛时向上位机发送 "arrive"
 *
 * 条件（同时满足）：
 *   1. X/Y 误差 < 20mm 且角度误差 < 3°
 *   2. 自身坐标 |X| >= 200 或 |Y| >= 200（远离原点，排除初始状态误判）
 *
 * 消抖：连续 10 tick 满足条件才发送（防止单次噪音触发）。
 * 动作运行期间不发送（s_chassis_send_cnt 清零）。
 */
static void chassis_TargetFilter(void)
{
    static int8_t s_chassis_send_cnt = 0;
  if (((Coord.target.z == 200 || Coord.target.z == 400 || Coord.target.z == 600) || (Coord.self.z == 200 && Coord.target.z == 0))
		&&VisData.Wp_Trk_Sig==1)
	{
			 if (((abs(Coord.target.x - Coord.self.x) < 150) &&
         (abs(Coord.target.y - Coord.self.y) < 150) &&
         abs(Coord.error.angle) < 10) &&
        (abs(Coord.self.x) >= 200 || abs(Coord.self.y) >= 200))
    {
        s_chassis_send_cnt++;
        if (s_chassis_send_cnt >= 50) // 连续 10 tick 满足 → 确认到达
        {
            s_chassis_send_cnt = 0;
            HAL_UART_Transmit(&huart6, kMsgArrive, 8, 100);
        }
    }
    else
        s_chassis_send_cnt = 0; // 不满足条件，重置计数
	}
	else{
    if (((abs(Coord.target.x - Coord.self.x) < 20) &&
         (abs(Coord.target.y - Coord.self.y) < 20) &&
         abs(Coord.error.angle) < 10) &&
        (abs(Coord.self.x) >= 200 || abs(Coord.self.y) >= 200))
    {
        s_chassis_send_cnt++;
        if (s_chassis_send_cnt >= 50) // 连续 10 tick 满足 → 确认到达 10
        {
            s_chassis_send_cnt = 0;
            HAL_UART_Transmit(&huart6, kMsgArrive, 8, 100);
        }
    }
    else
        s_chassis_send_cnt = 0; // 不满足条件，重置计数
	}
    // 动作运行期间不发送 arrive
    if (RC_R2.Act.now.task_state == kRunning)
    {
        s_chassis_send_cnt = 0;
    }
}

/**
 * @brief 动作启动条件判断 — 对准精度达标后从 Ready 升级为 Running
 *
 * 三个独立判断分支：
 *
 * 1. 抓取武器头（Wp_Trk_Sig=3）：
 *    X/Y 误差 < 40mm → 连续 5 tick → 升级为 kRunning
 *
 * 2. 夹取失败重试（Kfs_Get_Sig=5）：
 *    X/Y 误差 < 20mm + 角度 < 5° → 连续 5 tick → 升级为 kRunning
 *    （同一个目标不需要重复进入入口，通过 state!=kRunning 过滤）
 *
 * 3. KFS 转移（Kfs_Get_Sig=6/7）：
 *    X/Y 误差 < 15mm + 角度 < 5° → 连续 5 tick → 升级为 kRunning
 *    异常兜底：角度 >45° 或位移 >200mm → 回退到 kReady 重新对准
 */
static void Action_start_should(void)
{
    static int8_t GetKFS_entery = 0;    // KFS 二次入口标记（防止重复进入）
    static int16_t s_wp_send_cnt = 0;   // 抓取对准计数
    static int16_t s_kfs_check_cnt = 0; // KFS 夹取对准计数
    static int16_t s_kfs_move_cnt = 0;  // KFS 转移对准计数

    // ===== 1. 抓取武器头对准判断 =====
    if (VisData.Wp_Trk_Sig == 3)
    {
        if (abs(Coord.error.x) < 40 && abs(Coord.error.y) < 40)
        {
            s_wp_send_cnt++;
            if (s_wp_send_cnt >= 20) // 连续 5 tick → 对准稳定
            {
                s_wp_send_cnt = 0;
                RobotAct_copy(&RC_R2, kActEvent_Grab, kRunning);
            }
        }
    }
    else
    {
        s_wp_send_cnt = 0;
    }

    // ===== 3. KFS 转移对准判断  =====
    if (VisData.Kfs_Get_Sig == 7)
    {   
        // 正常对准 → 升级为 Running
        if (abs(Coord.error.x) < 150 &&
            abs(Coord.error.y) < 150 &&
            abs(Coord.target.angle) < 10)
        {
            s_kfs_move_cnt++;
            if (s_kfs_move_cnt >= 5)
            {
                s_kfs_move_cnt = 0;
                GetKFS_entery = 0;
                RobotAct_copy(&RC_R2, kActEvent_KfsMove, kRunning);
            }
        }
    }
    else
    {
        s_kfs_move_cnt = 0;
    }
}
