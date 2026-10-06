/**
 ******************************************************************************
 * @file    CommandTask.c
 * @brief   串口通信任务 — 接收视觉上位机数据并解析到全局变量
 * @details 通过 UART6 DMA 接收 49 字节定长数据包（帧头 0xFF 0xFF + 帧尾 0xFC），
 *          解析出机器人坐标、视觉事件标志位等，写入全局 Coord / VisData。
 *          任务由信号量 sem_commandHandle 驱动，DMA 收到完整帧后释放信号量。
 * @date    2026/5/9
 * @version V2.2
 ******************************************************************************
 */

#include "CommandTask.h"
#include "rob_core.h"
#include "usart_send.h"

/*
 * 视觉事件标志位 (Vs_BombSignals) 值含义速查
 *
 * Wp_Trk_Sig 武器点跟踪:
 *   0 = 无目标
 *   1 = 对准中·初级 → brain:Grab kReady
 *   2 = 对准中·精确 → brain:Grab kReady, chs:AutoTrack(中速0.7)
 *   3 = 就绪可抓取 → Action_start_should 误差<40mm连续5tick→升级Grab kRunning
 *
 * Kfs_Get_Sig KFS夹取:
 *   0 = 无目标
 *   1 = 初始化龙门架 → brain:GantryGet kInit, chs:Idle(中速)
 *   2 = 梅林准备夹取 → brain:GantryGet kReady, chs:ApproachKFS
 *   3 = 平地夹取·准备 → brain:GantryGet kReady, chs:AutoTrack+Idle(中速)
 *   4 = 平地夹取·执行 → brain:GantryGet kRunning(持mutex_ActHandle锁)
 *   5 = 夹取失败·重试 → brain:GantryGet kReady, chs:AutoTrack
 *       Action_start_should:误差<20mm+角<5°连续5tick→升级kRunning
 *   6 = 转移KFS·夹取 → brain:KfsMove kReady, chs:ApproachKFS
 *       Action_start_should:误差<15mm+角<5°→升级kRunning
 *   7 = 转移KFS·释放 → brain:KfsMove kReady, chs:ApproachKFS(龙门架+20mm避让)
 *   8 = 梅林自动对准 → chs:AutoTrack(仅底盘,不触发动作)
 *   9 = 梅林直接夹取 → brain:GantryGet kRunning, chs:Stop
 *
 * Climb_Sig 爬升:
 *   0 = 无
 *   1 = 开始爬坡 → chs:Climb
 *
 * Asm_Trk_Sig 装配:
 *   0 = 无
 *   1 = 执行装配 → brain:Assembly kRunning(夹爪张开→320ms→归中→finish)
 *
 * Kfs_Put_Sig KFS放置:
 *   0 = 无
 *   1 = 准备放置 → brain:GantryPut kReady(视觉扫描+龙门架就位)
 *   2 = 执行放置 → brain:GantryPut kRunning(夹持30°+推杆105mm→气阀释放→归位)
 *
 * Combine_Sig 合体:
 *   0 = 无
 *   1 = 合体对准 → chs:AutoTrack, act:龙门架保持150mm避让
 *   2 = 合体上R1 → chs:Combine, 完成后g_combine_achieve=1→brain保持Stop
 *   3 = 合体下R1 → chs:Combine, g_combine_achieve=0
 *
 * Speed 速度档位:
 *   0 = 全速+ (coeff=1.2)
 *   1 = 全速  (coeff=1.0)
 *   2 = 停止  (coeff=0.0)
 *
 * 底盘决策优先级: Stop > StepUpDown > Combine > Climb > AutoTrack > ApproachKFS > Idle
 * 例外: 动作Running/合体完成/Kfs=9→强制Stop, RobotProtect→强制Stop, Mux_updown=0→跳过决策
 */

// ==================== 全局数据实例 ====================

// 视觉事件标志位数据包 — 上位机解析视觉传感器后下发的识别结果
Vs_BombSignals VisData;

// 全局坐标系实例 — 所有模块通过 Coord.xxx 获取位置信息
CoordinateSystem Coord;

VerifySignals Verify;

/**
 * @brief 串口命令接收任务 — 解析上位机 49 字节数据包
 *
 * 数据包格式（小端序，索引从 0 开始）：
 *   [0-1]   帧头：0xFF 0xFF
 *   [2-3]   Coord.self.x       — 机器人当前 X 坐标 (mm)
 *   [4-5]   Coord.self.y       — 机器人当前 Y 坐标 (mm)
 *   [6-7]   Coord.self.z       — 机器人当前 Z 坐标 (mm)
 *   [8-9]   Coord.target.x     — 目标 X 坐标 (mm)
 *   [10-11] Coord.target.y     — 目标 Y 坐标 (mm)
 *   [12-13] Coord.target.z     — 目标 Z 坐标 (mm)
 *   [14-15] Coord.self.angle   — 当前朝向角 (度)
 *   [16-17] Coord.target.angle — 目标朝向角 (度)
 *   [18-19] Coord.grid.x       — 九宫格 X 坐标
 *   [20-21] Coord.grid.y       — 九宫格 Y 坐标
 *   [22-23] Coord.grid.z       — 九宫格 Z 坐标
 *   [24-25] Coord.kfs.x        — KFS 组件 X 坐标
 *   [26-27] Coord.kfs.y        — KFS 组件 Y 坐标
 *   [28-29] Coord.kfs.z        — KFS 组件 Z 坐标
 *   [30-31] VisData.Wp_Trk_Sig  — 武器点跟踪信号
 *   [32-33] VisData.Kfs_Get_Sig — KFS 夹取信号
 *   [34-35] VisData.Climb_Sig   — 爬升信号
 *   [36-37] Coord.detect.x     — 检测点 X 坐标
 *   [38-39] Coord.detect.y     — 检测点 Y 坐标
 *   [40-41] VisData.Asm_Trk_Sig — 装配跟踪信号
 *   [42-43] VisData.Kfs_Put_Sig — KFS 放置信号
 *   [44-45] VisData.Combine_Sig — 合体信号
 *   [46-47] VisData.Speed       — 速度档位
 *   [48]    帧尾：0xFC
 *
 * @note 任务由 sem_commandHandle 信号量阻塞等待，DMA 接收完成 ISR 中释放。
 *       上位机未连接时任务永久阻塞，不会空转消耗 CPU。
 */
void StartCommandTask(void const *argument)
{
    UART6_Receive_Start();      // 启动 DMA 接收（循环模式，49 字节缓冲区）
    for (;;)
    {   
        osSemaphoreAcquire(sem_commandHandle, osWaitForever);
        // 验证帧头 (0xFF 0xFF) 与帧尾 (0xFC) — 防止错位解析
        if (Rx_Data[0] == 0xFF && Rx_Data[1] == 0xFF && Rx_Data[50] == 0xFC)
        {
            // === 机器人自身坐标系 ===
            Coord.self.x = Rx_Data[3] << 8 | Rx_Data[2];
            Coord.self.y = Rx_Data[5] << 8 | Rx_Data[4];
            Coord.self.z = Rx_Data[7] << 8 | Rx_Data[6];

            // === 目标坐标系 ===
            Coord.target.x = Rx_Data[9] << 8 | Rx_Data[8];
            Coord.target.y = Rx_Data[11] << 8 | Rx_Data[10];
            Coord.target.z = Rx_Data[13] << 8 | Rx_Data[12];

            // === 朝向角度 ===
            Coord.self.angle = Rx_Data[15] << 8 | Rx_Data[14];
            Coord.target.angle = Rx_Data[17] << 8 | Rx_Data[16];

            // === 九宫格参考点 ===
            Coord.grid.x = Rx_Data[19] << 8 | Rx_Data[18];
            Coord.grid.y = Rx_Data[21] << 8 | Rx_Data[20];
            Coord.grid.z = Rx_Data[23] << 8 | Rx_Data[22];

            // === KFS 组件坐标（angle 沿用 target.angle） ===
            Coord.kfs.x = Rx_Data[25] << 8 | Rx_Data[24];
            Coord.kfs.y = Rx_Data[27] << 8 | Rx_Data[26];
            Coord.kfs.z = Rx_Data[29] << 8 | Rx_Data[28];
            Coord.kfs.angle = Coord.target.angle;

            // === 视觉事件标志位 ===
            VisData.Wp_Trk_Sig  = Rx_Data[31] << 8 | Rx_Data[30];
            VisData.Kfs_Get_Sig = Rx_Data[33] << 8 | Rx_Data[32];
            VisData.Climb_Sig   = Rx_Data[35] << 8 | Rx_Data[34];

            // === 检测点坐标（用于 AutoTrack 视觉对准） ===
            Coord.detect.angle = Coord.target.angle;
            Coord.detect.x = Rx_Data[37] << 8 | Rx_Data[36];
            Coord.detect.y = Rx_Data[39] << 8 | Rx_Data[38];

            // === 更多视觉事件标志位 ===
            VisData.Asm_Trk_Sig   = Rx_Data[41] << 8 | Rx_Data[40];
            VisData.Kfs_Put_Sig   = Rx_Data[43] << 8 | Rx_Data[42];
            VisData.Combine_Sig   = Rx_Data[45] << 8 | Rx_Data[44];
            VisData.Speed         = Rx_Data[47] << 8 | Rx_Data[46];
						 
						Verify.LED1_Sig = Rx_Data[49] << 8 | Rx_Data[48];
        }
    }
}
