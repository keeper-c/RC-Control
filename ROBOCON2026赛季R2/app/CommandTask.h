#ifndef __COMMANDTASK_H
#define __COMMANDTASK_H

// ===================================================================
// 分层：CommandTask — 应用层-通信协议
//   定义视觉上位机与机器人之间的数据协议结构体，
//   包括坐标系统（CoordinateSystem）和视觉视觉事件标志位（Vs_BombSignals）。
//   上位机通过 UART6 以 49 字节定长包发送数据，
//   CommandTask 任务接收并解析后写入全局 Coord / VisData。
// ===================================================================

#include "main.h"
#include "usart_send.h"

// 坐标结构体 — 三维空间中的一个点（带朝向）
typedef struct
{
    int16_t x;      // X 坐标（mm，场地坐标系）
    int16_t y;      // Y 坐标（mm，场地坐标系）
    int16_t z;      // Z 坐标（mm，高度）
    int16_t angle;  // 朝向角度（度，0=正前方）
} Coordinate;

// ===================================================================
// 视觉事件标志位数据包 — 上位机下发的视觉识别结果
// 由 CommandTask 解析 Rx_Data 后写入，brain / chs / act 读取做决策。
//
// 每个标志位的值含义详见下方字段注释。
// ===================================================================
typedef struct Vs_BombSignals
{
    volatile int8_t R1_signal_Sig;   // R1 信号标志（预留）

    // 武器点跟踪信号 — 视觉锁定武器弹药位置
    //   0 = 无目标（不触发）
    //   1 = 对准中·初级阶段 — brain→Grab kReady（准备夹取）
    //   2 = 对准中·精确阶段 — brain→Grab kReady, chs→AutoTrack（中速 0.7 慢速对准）
    //   3 = 就绪可抓取      — brain→Grab kRunning（由 Action_start_should 误差<40mm 连续5tick升级）
    volatile int8_t Wp_Trk_Sig;

    // KFS夹取信号 — 龙门架夹取关键功能组件的全流程信号
    //   0 = 无目标
    //   1 = 初始化龙门架   — brain→GantryGet kInit, chs Idle 中速 0.7
    //   2 = 准备夹取       — brain→GantryGet kReady, chs→ApproachKFS（靠近基板）
    //   3 = 平地夹取·准备  — brain→GantryGet kReady, chs→AutoTrack + Idle 中速
    //   4 = 平地夹取·执行  — brain→GantryGet kRunning（持锁执行）
    //   5 = 夹取失败·重试  — brain→GantryGet kReady（保持BatchID不变，重复夹取）, chs→AutoTrack
    //                       Action_start_should 检测误差<20mm+角度<5°连续5tick→升级kRunning
    //   6 = 转移KFS·夹取模式 — brain→KfsMove kReady, chs→ApproachKFS
    //                         Action_start_should 检测误差<15mm+角度<5°→升级kRunning
    //   7 = 转移KFS·释放模式 — brain→KfsMove kReady, chs→ApproachKFS（直接释放，龙门架升高20mm避让）
    //   8 = 备用自动对准   — chs→AutoTrack（仅触发底盘对准，不触发动作）
    //   9 = 特殊·直接夹取  — brain→GantryGet kRunning, chs→Stop（跳过准备阶段直接执行）
    volatile int8_t Kfs_Get_Sig;

    // 爬升信号 — 触发爬坡/垂直攀爬
    //   0 = 无
    //   1 = 开始爬升 — chs→Climb（朝目标全速前进）
    volatile int8_t Climb_Sig;

    // 装配跟踪信号 — 将弹药装入发射机构
    //   0 = 无
    //   1 = 执行装配 — brain→Assembly kRunning（夹爪张开→等320ms→归中→finish）
    volatile int8_t Asm_Trk_Sig;

    // KFS放置信号 — 将KFS放置到九宫格目标位
    //   0 = 无
    //   1 = 准备放置 — brain→GantryPut kReady（视觉扫描+龙门架就位）
    //   2 = 执行放置 — brain→GantryPut kRunning（夹持30°+推杆105mm→气阀释放→归位）
    volatile int8_t Kfs_Put_Sig;

    // 合体信号 — 两台机器人对接组合
    //   0 = 无
    //   1 = 合体对准   — chs→AutoTrack（视觉自动对准目标机器人）
    //                   act 龙门架保持 150mm 安全高度避让
    //   2 = 合体上坡   — chs→Combine(kStepClimb_Up_combine)
    //                   履带 75/138mm+麦轮±10.4→归零→g_combine_achieve=1
    //   3 = 合体下坡   — chs→Combine(kStepClimb_Done_combine), brain→g_combine_achieve=0
    //                   履带 70/136mm+麦轮±7.6→30/30mm→完成
    volatile int8_t Combine_Sig;

    // 速度档位 — 控制底盘导航速度系数
    //   0 = 全速+ (coeff=1.2) — 高速抵达目标
    //   1 = 全速  (coeff=1.0) — 正常速度
    //   2 = 停止  (coeff=0.0) — 麦轮停转，仅履带保持归位
    volatile int8_t Speed;
} Vs_BombSignals;

// 全局视觉信号实例 — CommandTask 写入，其他模块只读
extern Vs_BombSignals VisData;

// 坐标系全局数据 — 包含机器人位姿、目标位姿、误差、外部参考点
// 由 CommandTask 解析上位机数据写入，chassis/action 决策读取。
typedef struct
{
    // === 机器人自身坐标系 ===
    Coordinate self;    // 机器人当前位置（robot_current）
    Coordinate target;  // 机器人目标位置（robot_target）
    Coordinate error;   // 位置误差 = target - self（robot_error）

    // === 外部参考坐标系 ===
    Coordinate grid;    // 九宫格参考点坐标（nine_square_grid）
    Coordinate kfs;     // KFS 关键组件坐标（距基板前方 kKfsForwardDist 推算）
    Coordinate detect;  // 视觉识别检测点（autoDetect，用于自动对准）

    // === 辅助坐标系 ===
    Coordinate zero;    // 零点参考（原点）
    Coordinate demo;    // 演示/临时目标点（ApproachWp 等状态使用）
    Coordinate inst;    // 指令坐标（预留）
    Coordinate offset;  // 坐标偏移量（校准用）
} CoordinateSystem;

typedef struct
{
    int8_t LED1_Sig;    
    int8_t LED2_Sig;  
    int8_t LED3_Sig;   

    int8_t LED4_Sig;    
    int8_t LED5_Sig;    
    int8_t LED6_Sig;  

}VerifySignals;

// 全局坐标系实例 — CommandTask 写入，其他模块只读
extern CoordinateSystem Coord;
extern VerifySignals Verify;

#endif
