/**
 ******************************************************************************
 * @file    rob_act.c
 * @brief   上层机构动作实现（Action_Function 任务）
 * @details 实现 GrabReady / Grab / Assembly / GantryGet / GantryPut / KfsMove
 *          共 6 个动作的 Init / Ready / Task 函数，
 *          注册到 rob_behave.c 的 task_pkg[][] → s_action_pkg[] 表中。
 *
 *          【执行框架】
 *          Action_Function 每 15ms tick 一次：
 *          1. 检测 now != last → 置 s_state_just_entered 标记
 *          2. 按 now.task_state 索引 task[state].execute() 调用对应阶段函数
 *          3. 夹爪臂根据动作类型自动旋转到位
 *          4. xTaskNotify 通知 brain 决策层 kRobEvt_Action
 *
 *          【持锁约定】
 *          GantryGet/KfsMove 的 Running 阶段持 mutex_ActHandle，
 *          完成后释放 → brain 可切动作。
 ******************************************************************************
 */
#include "rob_act.h"

#define meilinqv 1  // 梅林区(1,2区)+挑战赛编译开关（1=梅林区参数）
#define duikangqv 0 // 对抗区(3区)编译开关（0=对抗区参数）

// 修改赛场为挑战赛/技能赛1，2区 或 技能赛3区
#define RobconType 1

// 动作完成消息 — uart_send_string 发送给上位机确认动作结束
static const char kMsgFinish[20] = "finish\r\n"; 

/* ========== 抓取/放置参数包结构体 ========== */
// GrabPkt_t — 夹取动作的完整参数集，由 GrabPkt_Config / PutPkt_Config 根据
//            BatchID（批次号）、TarH（目标高度层）、视觉信号查表填充。
//            复用为放置动作的参数包（放置时通过 PutPkt_Config 覆盖）。
typedef struct 
{   
    uint32_t magic; // 看门狗标志位（预留，未启用）

    int8_t BatchID; // 批次号：0x00=第一趟, 0x01=第二趟, 0x02=第三趟
                    // 影响 LiftFinishPos（不同趟次放置高度不同）

    int16_t TarH; // 目标高度层差值 = Coord.kfs.z - Coord.self.z
                  // 可能值：-200 / 200 / 400，决定各段运动参数

    int32_t FrontDelay; // 第一阶段附加延时（ms）— 夹取前缓冲
    int32_t RearDelay;  // 后段附加延时（ms）— 退出前缓冲
    int32_t BatchDelay; // 批次附加延时（ms）— 多趟次间隔
    int32_t SumDelay;   // 总附加延时 = FrontDelay + RearDelay + BatchDelay

    float LiftStartPos;  // 龙门架起始高度（mm）
    float LiftFinishPos; // 龙门架目标高度（mm）
    float PushStartPos;  // 推杆起始位置（mm）
    float GrpAng;        // 夹取角度（度）— 夹爪张合角度

    float LiftSpd;       // 龙门架升降速度
    float PushSpd;       // 推杆伸缩速度
    float GrpSpd;        // 夹取速度
    float RetreatRunSpd; // 夹爪回退速度（夹取后的慢速释放）

    int8_t spd_direction; // 速度方向标志（预留）
} GrabPkt_t;

// 全局抓取参数实例 — 整个任务共用，每次动作进入时重新配置
static GrabPkt_t s_grab_pkt;

/***************************************************************
 * 静态辅助函数声明
 ****************************************************************/
static void GrabPkt_Config(GrabPkt_t *pkt); // 夹取动作参数配置
static void PutPkt_Config(GrabPkt_t *pkt);  // 放置动作参数配置

// ---- 状态首次进入/退出标记 ----
// s_state_just_entered: 非零表示当前动作刚切换（now != last），
//   动作 task 函数在首次 tick 检测此标记，执行一次性初始化后清零。
// s_state_just_exited: 非零表示动作刚退出，用于发送 finish 消息等收尾操作。
static int8_t s_state_just_entered = 1;
static int8_t s_state_just_exited = 1;
static int8_t s_Mutex_just_exited = 1;
// PID 输出缓存 — 用于底盘电机速度闭环控制
float s_pid_posout[4]; // 位置环 PID 输出
float s_pid_spdout[4]; // 速度环 PID 输出

int8_t Grab_count = 0;
int8_t yawFirst = 0;
int8_t fellowFirst = 0;
int8_t Grab_done = 0;

/**
 * @brief 上层机构动作执行任务 — 所有动作的统一调度循环
 *
 * 该任务是整个上层机构动作系统的运行时入口：
 * - 每 15ms 执行一次当前动作的当前阶段（task[state].execute）
 * - 检测动作切换 → 置首次进入标记
 * - 根据动作类型控制夹爪臂旋转角度
 * - 非 GantryGet 期间保持底盘电机速度 PID 闭环
 * - 完成后通知 brain 决策层更新状态
 */
void Action_Function(void const *argument)
{
    s_grab_pkt = (GrabPkt_t){0}; // 结构体清零 — 确保参数从已知状态开始
    static int8_t combine_entry = 0;

#if RobconType
    // 技能赛--1，2区 / 挑战赛
    s_grab_pkt.BatchID = 0x00;
#else
    // 技能赛--对抗区
    s_grab_pkt.BatchID = 0x01;
#endif

    for (;;)
    {    
        /*********************动作状态机执行层*************************/
        // 检测动作或任务状态是否发生变化（brain 决策层写入了新动作）
        if (RC_R2.Act.now.task_state != RC_R2.Act.last.task_state ||
            RC_R2.Act.now.id != RC_R2.Act.last.id)
        {
            s_state_just_entered = 1; // 触发一次性初始化
            s_state_just_exited = 1;  // 触发退出收尾
            s_Mutex_just_exited = 1;
            // 同步 last → now，下一 tick 不再触发
            RC_R2.Act.last = RC_R2.Act.now;
        }

        /************************动作执行层*******************************/
        // 按 task_state 索引当前阶段的 execute 函数并调用
        // task[0]=Idle(NULL), task[1]=Init, task[2]=Ready, task[3]=Running
        int8_t state = RC_R2.Act.now.task_state;
        if (RC_R2.Act.now.task[state].execute != NULL)
        {
            RC_R2.Act.now.task[state].execute();
        }

        /***********************夹爪臂旋转****************************/
        // Grab 动作：夹爪臂归零（对准抓取位置）
        // 非 Grab 动作：夹爪臂偏转 2rad （竖起来）
        if (RC_R2.Act.now.id == kActEvent_Grab)
        {
            RC_R2.Act.power->Grip_arm(1.9f, 5.0f);
        }
        else
        {
            RC_R2.Act.power->Grip_arm(0.0f, 5.0f);
        }

        // 非 GantryGet 期间：保持底盘电机速度 PID 闭环（静止锁定）
        if (RC_R2.Act.now.id != kActEvent_GantryGet)
        {
            s_pid_spdout[1] = PID_Calc(&pid_DJI_v2, 0, motor_chassis[1].speed_rpm);
            s_pid_spdout[2] = PID_Calc(&pid_DJI_v3, 0, motor_chassis[2].speed_rpm);
            Can1_Sendmassage_classis(0, s_pid_spdout[1], s_pid_spdout[2], 0);
        }

        // 合体信号期间：龙门架保持 150 高度（给视觉无遮挡的扫描高度）
        if (VisData.Combine_Sig == 4 && VisData.Kfs_Put_Sig == 0)
        {
            if (combine_entry == 1)
            {
                if (s_grab_pkt.BatchID == 0x02)
                {
                    HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET);
                    RC_R2.Act.power->Gantry_lift(150.0f, 5.0f);
                    osDelay(200);
                    HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_SET);
                    combine_entry = 0;
                }
                else
                    RC_R2.Act.power->Gantry_lift(150.0f, 5.0f);
            }
        }
        else
        {
            combine_entry = 1;
        }

        /*校验视觉雷达相机是否启动完毕*/
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

        if (Verify.LED1_Sig == 1)
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

        // 通知 brain 决策层：动作执行层已完成本轮 tick
        xTaskNotify(StateFlag_TaskHandle, kRobEvt_Action, eSetBits);
        vTaskDelay(pdMS_TO_TICKS(15)); // 15ms 控制周期
    }
}

/* ===================================================================
 * 上层动作实现
 * =================================================================== */

/**
 * @brief 抓取准备 — 龙门架抬升至安全高度 + 夹爪张开
 *
 * 时序：
 *   0~320ms:  龙门架升到 108，夹爪张开（PWM 1900）
 *   320ms+:   夹爪归中（PWM 1500），龙门架保持 108
 */
void Act_GrabReady(void)
{   
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        s_action_timer = osKernelGetTickCount(); // 记录动作起始时刻
        s_state_just_entered = 0;
    }
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;
    RC_R2.Act.power->Gantry_lift(104.0f, 5.0f);

    if (Action_elapsed <= 320)
    {   
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1850); // 夹爪张开
    }   
    else
    {
        if (s_state_just_exited)
        {
            s_state_just_exited = 0;
        }
    }
    fellowFirst = 1;
    yawFirst = 0;
}

/**
 * @brief 抓取执行 — 夹取 → 抬起 → 后退 → 完成
 *
 * 时序（5 段）：
 *   0~800ms:   夹爪闭合（PWM 1000），龙门架 108
 *   800~1050ms: 龙门架升到 140mm（抬起弹药）
 *   1050~1250ms: 底盘后退 12rpm × 4 轮（脱离抓取位）
 *   1250ms+:    夹爪归中，龙门架归零，底盘停转，发送 "finish\r\n"
 */
void Act_GrabTask(void)
{   
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        Grab_count++;
        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;

    if (Action_elapsed <= 1100)
    {
        RC_R2.Act.power->Gantry_lift(104.0f, 5.0f);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1300); // 夹紧
    }
    else if (Action_elapsed > 1100 && Action_elapsed <= 1350)
    {
        RC_R2.Act.power->Gantry_lift(140.0f, 5.0f); // 抬起
    }
    else if (Action_elapsed > 1350 && Action_elapsed <= 1550)
    {
        RC_R2.Chs.power->Wheel(-7, -7, 7, 7, 1); // 后退脱离
    }
    else if (Action_elapsed > 1550)
    {   
        RC_R2.Act.power->Gantry_lift(0.0f, 5.0f); // 龙门架归零
        if (s_state_just_exited)
        {   
            RC_R2.Chs.power->Wheel(0, 0, 0, 0, 0); // 底盘停转
            uart_send_string(kMsgFinish);          // 通知上位机完成
            s_state_just_exited = 0;
        }   
    }       
		Grab_done=1;
}

/**
 * @brief 装配执行 — 夹爪推入弹药 
 * 
 * 时序：
 *   夹爪张开（PWM 1900）→ 等 320ms → 夹爪归中（PWM 1500）→ 发送 finish
 */
void Act_AssemblyTask(void)
{
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;
    if (Action_elapsed <= 1020)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2200); // 夹爪张开
    }
    else
    {
        if (s_state_just_exited)
        {
            uart_send_string(kMsgFinish);
            s_state_just_exited = 0;
        }
    }
    yawFirst = 1;
		Grab_done = 0;
}

/**
 * @brief 龙门架夹取初始化 — 龙门架预就位到 110
 */
void Act_GantryGetInit(void)
{
    RC_R2.Act.power->Gantry_lift(110.0f, 5.0f);
    
    if (s_state_just_entered)
    {
        s_state_just_entered = 0;
    }
		yawFirst = 1;
}

/**
 * @brief 龙门架夹取准备 — 配置参数 + 移动到起始位置
 */
void Act_GantryGetRdy(void)
{
    GrabPkt_Config(&s_grab_pkt);                                 // 查表填充参数
    RC_R2.Act.power->Gantry_lift(s_grab_pkt.LiftStartPos, 5.0f); // 龙门架就位
    RC_R2.Act.power->Gantry_push(s_grab_pkt.PushStartPos, 6.0f); // 推杆就位
}

/**
 * @brief 龙门架夹取执行 — 多段时序控制完整夹取过程
 *
 * 夹取流程（共 7 段时序）：
 *   段1 (0~350+FrontDelay):         快速夹取（9rad/s）
 *   段2 (350~450+FrontDelay):       慢速收紧（3rad/s）
 *   段3 (450~600+FrontDelay):       气阀上升 + 回退速度夹紧 + 龙门架升到目标高度
 *   段4 (600~1500+SumDelay):        龙门架继续上升 + 夹紧保持
 *   段5 (1500~1900+SumDelay):       龙门架保持 + 慢速夹紧
 *   段6 (1900~2000+SumDelay):       气阀关闭
 *   段7 (2000~3000+SumDelay):       推杆前伸  + 履带反向压紧
 *   段8 (3000~3800+SumDelay):       推杆归零 + 履带归零
 *   段9 (3800+SumDelay):            完成收尾：第三趟夹取时龙门架升至 135
 *
 * @note 持 mutex_ActHandle 运行，完成后释放。
 *       Kfs_Get_Sig=5 时 BatchID++（记录第几次重复夹取）。
 */
void Act_GantryGetTask(void)
{   
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        GrabPkt_Config(&s_grab_pkt);
        if (VisData.Kfs_Get_Sig != 5)
        {   
            // 非重复夹取：批次号 +1（首次 0→1，第二次 1→2）
            s_grab_pkt.BatchID++;
        }   
        if (VisData.Kfs_Get_Sig == 5)
        {
            // 夹取失败重复：保持 BatchID 不变，稍作缓冲防止准备时间不足
            osDelay(300);
        }
        osMutexAcquire(mutex_ActHandle, 0); // 持锁 — 夹取期间不允许切动作
        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }

    // 简洁别名 — 避免重复写 &s_grab_pkt
    GrabPkt_t *pkt = &s_grab_pkt;
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;

    // ===== 段1: 快速夹取 =====
    if (Action_elapsed <= 350 + pkt->FrontDelay)
    {
        RC_R2.Act.power->Gantry_grip(pkt->GrpAng, 9.0f);
    }
    // ===== 段2: 慢速收紧 =====
    else if (Action_elapsed > 350 + pkt->FrontDelay &&
             Action_elapsed <= 450 + pkt->FrontDelay)
    {
        RC_R2.Act.power->Gantry_grip(pkt->GrpAng, 3.0f);
    }
    // ===== 段3: 气阀上升 + 夹紧回退 + 龙门架上升 =====
    else if (Action_elapsed > 450 + pkt->FrontDelay &&
             Action_elapsed <= 650 + pkt->FrontDelay)
    {   
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_SET); // 气阀升起
        RC_R2.Act.power->Gantry_grip(pkt->GrpAng, pkt->RetreatRunSpd);
        RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 6.0f);
    }   
    // ===== 段4: 龙门架持续上升 + 夹紧保持 =====
    else if (Action_elapsed > 650 + pkt->FrontDelay &&
             Action_elapsed <= 1550 + pkt->SumDelay)
    {
        RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 6.0f);
        RC_R2.Act.power->Gantry_grip(0.0f, pkt->RetreatRunSpd);
    }
    // ===== 段5: 慢速夹紧 =====
    else if (Action_elapsed > 1550 + pkt->SumDelay &&
             Action_elapsed <= 1950 + pkt->SumDelay)
    {
        RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 6.0f);
        RC_R2.Act.power->Gantry_grip(0.0f, 2.0f);
    }
    // ===== 段6: 气阀关闭 =====
    else if (Action_elapsed > 1950 + pkt->SumDelay &&
             Action_elapsed <= 2050 + pkt->SumDelay)
    {
        RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 6.0f);
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET);
    }
    else if (Action_elapsed > 2050 + pkt->SumDelay &&
             Action_elapsed <= 2250 + pkt->SumDelay)
    {
        // 等KFS秘籍掉下去
    }
    // ===== 段7: 推杆前伸 + 2006电机转动规整（固定 KFS 位置） =====
    else if (Action_elapsed > 2250 + pkt->SumDelay &&
             Action_elapsed <= 3950 + pkt->SumDelay)
    {
        s_pid_spdout[1] = PID_Calc(&pid_DJI_v2, 5000, motor_chassis[1].speed_rpm);
        s_pid_spdout[2] = PID_Calc(&pid_DJI_v3, -5000, motor_chassis[2].speed_rpm);
        Can1_Sendmassage_classis(0, s_pid_spdout[1], s_pid_spdout[2], 0);
    }
    // ===== 段8: 推杆归零 + 2006电机归零 =====
    else if (Action_elapsed > 3950 + pkt->SumDelay &&
             Action_elapsed <= 4150 + pkt->SumDelay)
    {   
        yawFirst = 1;
        s_pid_spdout[1] = PID_Calc(&pid_DJI_v2, 0, motor_chassis[1].speed_rpm);
        s_pid_spdout[2] = PID_Calc(&pid_DJI_v3, 0, motor_chassis[2].speed_rpm);
        Can1_Sendmassage_classis(0, s_pid_spdout[1], s_pid_spdout[2], 0);
    }
    // ===== 段9: 完成收尾 =====
    else if (Action_elapsed > 4150 + pkt->SumDelay)
    {
        Can1_Sendmassage_classis(0, 0, 0, 0);
        RC_R2.Act.power->Gantry_push(0.0f, 6.0f);

        // 第2趟夹取：龙门架额外升到 135（为放置做准备）
        if (s_grab_pkt.BatchID == 0x02)
        {
            RC_R2.Act.power->Gantry_lift(135.0f, 5.0f);
            osDelay(1500);
            HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_SET);
        }

        if (s_state_just_exited)
        {
            uart_send_string(kMsgFinish);
            osMutexRelease(mutex_ActHandle); // 释放互斥锁 → brain 可切动作
            s_state_just_exited = 0;
        }
    }
}

/**
 * @brief KFS 转移准备 — 根据视觉信号判断移动模式
 *
 * Kfs_Get_Sig == 6: 龙门架就位准备夹取
 * Kfs_Get_Sig == 7: 龙门架升高 20（避让），推杆归零（直接释放模式）
 */
void Act_KfsMoveRdy(void)
{
    
    GrabPkt_Config(&s_grab_pkt);
    if (VisData.Kfs_Get_Sig == 10)
    {
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET); // 气阀关闭
        // 夹取模式：龙门架和推杆就位
        RC_R2.Act.power->Gantry_lift(s_grab_pkt.LiftStartPos, 5.0f);
        RC_R2.Act.power->Gantry_push(s_grab_pkt.PushStartPos, 6.0f);
    }
    else if (VisData.Kfs_Get_Sig == 7)
    {
        // 释放模式：龙门架升高 20mm，推杆归零
        RC_R2.Act.power->Gantry_lift(s_grab_pkt.LiftStartPos + 40.0f, 5.0f);
        RC_R2.Act.power->Gantry_push(7.0f, 6.0f);
    }
}

/**
 * @brief KFS 转移执行 — 按视觉信号分支执行
 *
 * Kfs_Get_Sig == 6（夹取模式）：
 *   350ms 快速夹 → 400ms 慢速 → 550ms 气阀 → 950ms 转为30° → 完成
 *
 * Kfs_Get_Sig == 7（释放模式）：
 *   450ms 59°夹紧 → 500ms 慢速 → 550ms 气阀关 → 完成后龙门架/推杆归零
 *
 * @note 持 mutex_ActHandle 运行，完成后释放。
 */
void Act_KfsMoveTask(void)
{
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        GrabPkt_Config(&s_grab_pkt);
        osMutexAcquire(mutex_ActHandle, 0); // 持锁
        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }
    GrabPkt_t *pkt = &s_grab_pkt;
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;

    // ===== 夹取模式 (Sig=6): 正常夹取 KFS =====
    if (VisData.Kfs_Get_Sig == 6)
    {
        if (Action_elapsed <= 350 + pkt->FrontDelay)
        {
            RC_R2.Act.power->Gantry_grip(pkt->GrpAng, 9.0f); // 快速夹取
        }
        else if (Action_elapsed > 350 + pkt->FrontDelay &&
                 Action_elapsed <= 450 + pkt->FrontDelay)
        {
            RC_R2.Act.power->Gantry_grip(pkt->GrpAng, 3.0f); // 慢速收紧
        }
        else if (Action_elapsed > 450 + pkt->FrontDelay &&
                 Action_elapsed <= 600 + pkt->FrontDelay)
        {
            HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_SET); // 气阀开
            RC_R2.Act.power->Gantry_grip(pkt->GrpAng, pkt->RetreatRunSpd);
        }
        else if (Action_elapsed > 600 + pkt->FrontDelay &&
                 Action_elapsed <= 1000 + pkt->SumDelay)
        {
            RC_R2.Act.power->Gantry_grip(30.0f, 3.0f); // 转为30°夹持
        }
        else if (Action_elapsed > 1000 + pkt->SumDelay)
        {
            if (s_state_just_exited)
            {
                uart_send_string(kMsgFinish);
                s_state_just_exited = 0;
                // 防止finish间隙
                osDelay(75);
                osMutexRelease(mutex_ActHandle);
            }
        }
    }
    // ===== 释放模式 (Sig=7): 直接释放 KFS =====
    else if (VisData.Kfs_Get_Sig == 7)
    {
        if (Action_elapsed <= 430)
        {
            RC_R2.Act.power->Gantry_grip(59.0f, 3.0f); // 快速夹紧59°
        }
        else if (Action_elapsed > 430 && Action_elapsed <= 460)
        {
            HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET); // 气阀关
        }
        else if (Action_elapsed > 460)
        {
            RC_R2.Act.power->Gantry_grip(0.0f, 3.0f);               // 夹爪归零
            RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 5.0f); // 龙门架归位
            RC_R2.Act.power->Gantry_push(0.0f, 5.0f);               // 推杆归零
            if (s_state_just_exited)
            {
                uart_send_string(kMsgFinish);
                s_state_just_exited = 0;
                osMutexRelease(mutex_ActHandle);
            }
            yawFirst = 1;
        }
    }
}

/* ===================================================================
 * 静态辅助函数
 * =================================================================== */

/**
 * @brief 夹取参数配置 — 根据 TarH、BatchID、视觉信号查表填充 GrabPkt_t
 *
 * 参数来源优先级：
 *   1. Kfs_Get_Sig 3/4（平地夹取）→ 固定参数
 *   2. Kfs_Get_Sig != 3/4（梅林上夹取）→ 按 TarH 查表
 *
 * TarH（目标高度差）分为三档：-200 / 200 / 400
 *   每档有不同的 LiftStartPos / GrpAng / RetreatRunSpd 等参数
 *
 * BatchID 影响 LiftFinishPos 和 BatchDelay：
 *   0x00（第一次）→ LiftFinishPos=89, BatchDelay=0
 *   0x01（第二次）→ LiftFinishPos=220, TarH=-200 时额外 +300ms
 */
static void GrabPkt_Config(GrabPkt_t *pkt)
{
    // ===== 非平地夹取（Kfs_Get_Sig != 3 且 != 4） =====
    if (VisData.Kfs_Get_Sig != 3 && VisData.Kfs_Get_Sig != 4)
    {
        pkt->TarH = Coord.kfs.z - Coord.self.z;

        // 按批次设定 LiftFinishPos
        if (pkt->BatchID == 0x00)
        {
            pkt->LiftFinishPos = 89.0f;
            pkt->BatchDelay = 0;
        }
        else if (pkt->BatchID == 0x01)
        {
            pkt->LiftFinishPos = 220.0f;
            if (pkt->TarH == -200)
            {
                pkt->BatchDelay = 300; // 负高度差额外延时
            }
            else
                pkt->BatchDelay = 0;
        }
        if (VisData.Kfs_Get_Sig == 7)
        {
            // 按批次设定 LiftFinishPos
            if (pkt->BatchID == 0x00)
            {
                pkt->LiftFinishPos = 89.0f;
            }
            else if (pkt->BatchID == 0x01)
            {
                pkt->LiftFinishPos = 89.0f;
            }
            else if (pkt->BatchID == 0x02)
            {
                pkt->LiftFinishPos = 220.0f;
            }
        }
        // 按 TarH 查表设定各段参数
        if (pkt->TarH == -200)
        {
            pkt->LiftStartPos = 0.0f;
            pkt->FrontDelay = 50;
            pkt->RearDelay = 600;
            pkt->GrpAng = 69.0f;
            pkt->RetreatRunSpd = 2.0f;
            pkt->PushStartPos = 7.0f;
        }
        else if (pkt->TarH == 200)
        {
            pkt->LiftStartPos = 90.0f;
            pkt->FrontDelay = 0;
            pkt->RearDelay = 0;
            pkt->GrpAng = 59.0f;
            pkt->RetreatRunSpd = 4.0f;
            pkt->PushStartPos = 7.0f;
        }
        else if (pkt->TarH == 400)
        {
            pkt->LiftStartPos = 180.0f;
            pkt->FrontDelay = 0;
            pkt->RearDelay = 100;
            pkt->GrpAng = 59.0f;
            pkt->RetreatRunSpd = 4.0f;
            pkt->PushStartPos = 7.0f;
        }
        else if (pkt->TarH == 0)
        {
            pkt->LiftStartPos = 50.0f;
            pkt->FrontDelay = 0;
            pkt->RearDelay = 0;
            pkt->GrpAng = 59.0f;
            pkt->RetreatRunSpd = 4.0f;
            pkt->PushStartPos = 7.0f;
        }

        pkt->SumDelay = pkt->FrontDelay + pkt->RearDelay + pkt->BatchDelay;
    }

    // ===== 平地夹取（Kfs_Get_Sig == 3 或 4） =====
    if (VisData.Kfs_Get_Sig == 3 || VisData.Kfs_Get_Sig == 4)
    {
        if (pkt->BatchID == 0x00)
        {
            pkt->LiftFinishPos = 59.0f;
            pkt->BatchDelay = 0;
        }
        else if (pkt->BatchID == 0x01)
        {
            pkt->LiftFinishPos = 220.0f;
            pkt->BatchDelay = 100;
        }
        pkt->LiftStartPos = 0.0f;
        pkt->FrontDelay = 0.0f;
        pkt->RearDelay = 100;
        pkt->GrpAng = 59.0f;
        pkt->RetreatRunSpd = 3.0f;
        pkt->PushStartPos = 7.0f;

        pkt->SumDelay = pkt->FrontDelay + pkt->RearDelay + pkt->BatchDelay;
    }
}

/**
 * @brief 放置准备 — 视觉扫描 + 龙门架就位
 *
 * 时序：
 *   0~1000ms: 按 BatchID 龙门架归零/推杆就位
 *   1000~1100ms: 气阀开启
 *   1100ms+: 龙门架升到目标起始高度
 */
void Act_GantryPutRdy(void)
{
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET);
        PutPkt_Config(&s_grab_pkt); // 查表配置放置参数
        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }
    GrabPkt_t *pkt = &s_grab_pkt;
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;
    
    if (Action_elapsed <= 2500)
    {
        // 第一趟放置 (BatchID=1): 龙门架 0 + 推杆 20
        // 第二趟放置 (BatchID=2): 龙门架 135 + 推杆 40
        switch (s_grab_pkt.BatchID)
        {
        case 0x01:
            RC_R2.Act.power->Gantry_lift(0.0f, 5.0f);
            RC_R2.Act.power->Gantry_push(1.3f, 10.0f);
            break;
        case 0x02:
            RC_R2.Act.power->Gantry_lift(135.0f, 5.0f);
            RC_R2.Act.power->Gantry_push(2.6f, 10.0f);
            break;
        }
    }
    else if (Action_elapsed > 2500 && Action_elapsed <= 2700)
    {
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_SET); // 气阀开
    }
    else if (Action_elapsed > 2700)
    {
        RC_R2.Act.power->Gantry_lift(160.0f, pkt->LiftSpd); // 龙门架就位
    }
}

/**
 * @brief 放置执行 — 推杆前伸 → 气阀释放 → 归位
 *
 * 时序：
 *   0~800ms:   夹爪 30° + 推杆 105（前推放置）
 *   800~1000ms: 气阀关闭（释放 KFS）
 *   1000ms+:    龙门架归零 + 夹爪归零 + 推杆归零 + 发送 finish
 *
 * @note 进入时 BatchID-- 因为是逆序放置（先放第三趟再放第二趟）
 */
void Act_GantryPutTask(void)
{
    static uint32_t s_action_timer;
    if (s_state_just_entered)
    {
        PutPkt_Config(&s_grab_pkt);
        // 逆序放置：第三趟→第二趟→第一趟（BatchID: 2 → 1 → 0）
        s_grab_pkt.BatchID--;

        s_action_timer = osKernelGetTickCount();
        s_state_just_entered = 0;
    }
    GrabPkt_t *pkt = &s_grab_pkt;
    uint32_t Action_elapsed = osKernelGetTickCount() - s_action_timer;
    
		if (Action_elapsed <= 500)
    {
        RC_R2.Act.power->Gantry_lift(pkt->LiftStartPos, pkt->LiftSpd); // 龙门架就位
    }
    else if (Action_elapsed > 500 && Action_elapsed <= 1700)
    {
        RC_R2.Act.power->Gantry_grip(40.0f, pkt->GrpSpd); // 夹持 30°
        RC_R2.Act.power->Gantry_push(7.0f, 10.0f);        // 推杆前伸 
    }
    else if (Action_elapsed > 1700 && Action_elapsed <= 1900)
    {
        HAL_GPIO_WritePin(qifa_up_GPIO_Port, qifa_up_Pin, GPIO_PIN_RESET); // 气阀关=释放
    }
    else if (Action_elapsed > 1900)
    {   
        RC_R2.Act.power->Gantry_lift(pkt->LiftFinishPos, 4.0f); // 龙门架归位
        RC_R2.Act.power->Gantry_grip(0.0f, 5.0f);               // 夹爪归零
        RC_R2.Act.power->Gantry_push(0.0f, 5.0f);               // 推杆归零
        if (s_state_just_exited)
        {
            uart_send_string(kMsgFinish);
            s_state_just_exited = 0;
        }
    }
}

/**
 * @brief 放置参数配置 — 根据 BatchID 查表填充 GrabPkt_t
 *
 * BatchID 2（第三趟，先放）: LiftStartPos=160, GrpSpd=5
 * BatchID 1（第二趟，后放）: LiftStartPos=180, GrpSpd=2, BatchDelay=200
 */
static void PutPkt_Config(GrabPkt_t *pkt)
{
    pkt->TarH = Coord.grid.z; // 以九宫格 Z 坐标为参考

    // 视觉合体状态
    if (VisData.Combine_Sig != 0)
    {
    }
    else if (VisData.Combine_Sig == 0)
    {
    }

    // BatchID == 2（第三趟 = 第一次放置）
    if (pkt->BatchID == 0x02)
    {
        pkt->FrontDelay = 0;
        pkt->BatchDelay = 0;
        pkt->LiftStartPos = 160.0f;
        pkt->LiftSpd = 5.0f;
        pkt->LiftFinishPos = 0.0f;
        pkt->GrpSpd = 5.0f;
    }
    // BatchID == 1（第二趟 = 第二次放置）
    else if (pkt->BatchID == 0x01)
    {
        pkt->FrontDelay = 0;
        pkt->BatchDelay = 200; // 额外等待 200ms
        pkt->LiftStartPos = 180.0f;
        pkt->LiftSpd = 10.0f; // 更快速度就位
        pkt->LiftFinishPos = 0.0f;
        pkt->GrpSpd = 2.0f; // 更慢夹持释放
    }
    pkt->SumDelay = pkt->FrontDelay + pkt->RearDelay + pkt->BatchDelay;
}
