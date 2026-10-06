/**
 * @file drv_DJ_Control.h
 * @brief DJI电机驱动头文件
 * @version 2.0
 * @date 2026-09-04
 * @author Ckeeper
 */
#ifndef __DJ_CONTROL_H__
#define __DJ_CONTROL_H__

/* ==================== 使用示例 ====================
  
   =================================================== */

#include "main.h"
#include "drv_can.h"
#include "alg_pid.h"

#define DJ_DATA_FRAME_SIZE 8
#define DJ_MOTOR_ADDR_0x200 0x200
#define DJ_MOTOR_ADDR_0x1ff 0x1ff

#define MOTOR_REDUCTION_RATIO   19.2f    // M3508
#define PI 3.1415926535f
#define DJI_ECD_TO_RAD      (2.0f * PI / 8192.0f)
#define RPM_TO_RAD_S        (2.0f * PI / 60.0f)
#define ECD_MAX             8191

/* ==================== 用户调参区(集中在此) ====================
   量纲约定(速度环): 输入 rad/s(输出轴) -> 输出 C620 电流raw(±16384),
   控制周期 1ms, dt 已折入 Ki/Kd, 本模块内部不感知时间。
   调参口诀: 响应慢->增大 Kp; 稳态有差->增大 Ki; 超调/振荡->减小 Kp/Ki/integral_max */
#define DJI_CURRENT_MAX  16384.0f    /* C620 电流满量程(raw), 输出限幅用 */
#define DJI_INTEGRAL_MAX 2000.0f     /* 速度环积分限幅(raw), 过冲上限≈I_MAX/KP≈2 rad/s */
#define KP   800.0f                  /* 速度环比例: raw/(rad/s) */
#define KI   1.0f                    /* 速度环积分: raw/(rad/s·tick), 过大瞬态灌满积分导致大过冲 */
#define KD   0.0f                   /* 速度环微分: raw/(rad/s) */

/* 位置环(串级外环): 反馈 rad(输出轴多圈角度), 输出目标速度 rad/s 给速度环 */
#define PKP  90.0f                    /* 位置环比例: (rad/s)/rad, 受扰回拉不顶格 */
#define PKI  0.000f                     /* 位置环积分: 串级外环用纯 P, 内环积分消稳态差; 加了容易振荡 */
#define PKD  1.5f                     /* 位置环微分: 接近目标时刹车, 抑制冲动 */
#define DJI_POSITION_INTEGRAL_MAX 150.0f /* 位置环积分限幅 */
#define DJI_POSITION_OUT_MAX 20.0f   /* 位置环输出限幅: 必须 < 电机空载极限(~25), 内环永远跟得上不饱和 */

/* ---- 位置模式: 在线梯形速度规划(参考 Motor.c 项目) ----
   v_allow = min(V_MAX, DEC_ENV_K·√(2·A_MAX·|s_rem|)), v_ref 斜坡逼近,
   pos_ref = v_ref 数值积分; 到位交棒看实际反馈(误差窗+速度窗)。
   调试: 冲击大→减小 A_MAX; 太慢→增大 V_MAX/A_MAX; 到位滑行远→增大 A_MAX */
#define DJI_CTRL_DT        0.001f     /* 控制周期(s), 须与 DJ_Motor_Control 实际调用周期一致 */
#define DJI_TRAJ_V_MAX     40.0f      /* 规划匀速上限(rad/s, 输出轴), 须小于空载极限(~25) */
#define DJI_TRAJ_A_MAX     40.0f      /* 规划加减速度(rad/s²), 决定起步/刹车柔性 */
#define DJI_DEC_ENV_K      0.55f      /* 减速包络余量系数, 提前减速消跟踪滞后 */
#define DJI_ARRIVE_POS_ERR 0.02f      /* 到位位置窗(rad, ≈1.1°), 看实际反馈 */
#define DJI_ARRIVE_VEL_MAX 1.0f       /* 到位速度窗(rad/s) */
#define DJI_PLAN_KFF       1.0f       /* 规划速度前馈: v_ref 直通叠加到速度目标 */

/* ---- 速度环增强 ---- */
#define DJI_SPD_FILT       0.85f      /* 速度反馈一阶低通系数(0~1, 越小越平滑; 过小滞后拖垮内环相位裕度) */

/* ---- 位置环 PID 增强(初始化时写入 position_pid 字段) ---- */
#define DJI_POS_SEP        0.087f     /* 积分分离窗(rad, ≈5°): 误差大时不积分防吹胀 */
#define DJI_POS_ALPHA      0.1f       /* 微分低通系数(微分先行对反馈微分) */



/**
 * @brief 大疆 C620 电调反馈数据帧(接收解析结果)
 * @note  反馈帧 ID = 0x200 + ID(1~8), 周期 1ms 由电调主动发送,
 *        包含转子位置(编码器)、转子转速(RPM)、实际电流、线圈温度。
 *        本驱动同时维护多圈累计与输出轴(减速后)量纲转换。
 */
typedef struct 
{
    uint32_t RX_ID;          /* 反馈帧 CAN ID = 0x200 + 电机ID(1~8) */
    int16_t  P_Des;         /* 转子单圈编码器原始值 0~8191 (C620 13位) */
    int16_t  V_Des;         /* 转子转速 RPM(转子侧, 未除减速比) */
    int16_t  Current;       /* 实际电流原始值(有符号) */
    uint8_t  Temperature;   /* 线圈温度(℃) */

    float mech_rad;         /* 转子单圈机械角度 0~2π rad (P_Des × 2π/8192) */
    float omega_rotor_rad_s;/* 转子角速度 rad/s (V_Des × 2π/60) */

    int16_t last_ecd;      /* 上一帧编码器值(用于多圈累计跨圈检测) */
    int32_t round_cnt;     /* 转子累计圈数(正/负, 多圈计数) */
    float total_rotor_rad; /* 转子多圈总弧度(累计, 不限 0~2π) */

    float reduction_ratio; /* 减速比: M3508=19.2, M2006=36.0 */
    float total_out_rad;   /* 输出轴多圈总弧度 = total_rotor_rad / 减速比 */
    float omega_out_rad_s; /* 输出轴角速度 rad/s = omega_rotor_rad_s / 减速比 */
    float omega_filt;      /* 低通后输出轴角速度 rad/s(速度环反馈用) */
}DJ_RX_Frame_t;

typedef enum{
    DJ_MOTOR_MODE_SPEED = 0,
    DJ_MOTOR_MODE_POSITION = 1,
}DJ_Motor_Mode_t;

typedef struct
{
    DJ_Motor_Mode_t mode;
    uint8_t ID;
    CAN_HandleTypeDef *hcan;
    int16_t current;
    uint8_t* data;
    DJ_RX_Frame_t RX_Frame;
    PID_t speed_pid;   /**< 速度环 PID 控制器(内环) */
    PID_t position_pid; /**< 位置环 PID 控制器(外环, 输出即内环目标速度) */

    /* 在线梯形速度规划(仅位置模式, DJ_Set_Target 启动) */
    uint8_t plan_sync;  /**< 首帧反馈同步标志: 同步规划轨迹到当前实际位置 */
    uint8_t plan_done;  /**< 到位交棒标志: 1=轨迹钉死目标, 规划结束 */
    float    tgt_pos;   /**< 目标位置(rad, 输出轴) */
    float    pos_ref;   /**< 规划位置轨迹(rad, 输出轴) */
    float    v_ref;     /**< 规划速度轨迹(rad/s, 输出轴) */
    float    plan_v_max;/**< 本次移动匀速上限(rad/s), 由 DJ_Set_Target 指定 */
}DJ_Motor_t;

extern uint8_t DJ_Send_Data_0x200[DJ_DATA_FRAME_SIZE]; /* 0x200 帧(ID 1~4)共享发送缓冲 */
extern uint8_t DJ_Send_Data_0x1ff[DJ_DATA_FRAME_SIZE]; /* 0x1ff 帧(ID 5~8)共享发送缓冲 */

/**
 * @brief 初始化电机对象
 * @param hcan            绑定的 CAN 外设
 * @param ID              电调 ID 1~8
 * @param mode            初始运行模式
 * @param reduction_ratio 减速比(M3508=19.2, M2006=36.0)
 * @param data_frame      指向该电机对应帧的共享发送缓冲区
 * @note  内部完成: 绑定外设、清零反馈、用 alg_pid.h 宏初始化两个 PID。
 *        位置模式死区默认禁用, 需要时在调用后写 pMotor->position_pid.deadband。
 */
void DJ_Motor_Init(DJ_Motor_t* pMotor, CAN_HandleTypeDef *hcan, uint8_t ID, DJ_Motor_Mode_t mode, float reduction_ratio, uint8_t* data_frame);

/**
 * @brief 直接设置电流(绕过 PID, 调试用)
 * @param current 电流 raw(±16384)
 */
void DJ_Set_Current(DJ_Motor_t* pMotor, int16_t current);

/**
 * @brief 设置目标(统一双参接口: 位置 + 速度)
 * @param pMotor  电机对象指针
 * @param pos     位置模式: 目标位置 rad(输出轴多圈); 速度模式: 忽略
 * @param speed   速度模式: 目标速度 rad/s(输出轴); 位置模式: 本次移动匀速上限
 *                rad/s(输出轴), 自动钳位到 [0.05, DJI_TRAJ_V_MAX]
 * @note  速度模式: speed 直给速度环, pos 被忽略。
 *        位置模式: 以 speed 为匀速上限启动在线梯形规划, 弧形接近目标,
 *        减速包络与加减速度仍由 DJI_DEC_ENV_K/DJI_TRAJ_A_MAX 决定;
 *        speed=DJI_TRAJ_V_MAX 即默认满速。
 */
void DJ_Set_Target(DJ_Motor_t* pMotor, float pos, float speed);

/**
 * @brief 电机控制总入口(在 TIM1 1ms 中断中调用)
 * @note  速度模式: 速度环 PID -> 电流;
 *        位置模式: 位置环(外环) -> 速度环(内环) -> 电流, 大疆 Demo 标准串级。
 *        运算后自动调用 DJ_Send_Frame 发送电流帧。
 */
void DJ_Motor_Control(DJ_Motor_t* pMotor);

/**
 * @brief 发送电流指令帧(C620 看门狗要求持续发送)
 * @note  ID 1~4 打包到 0x200 帧, ID 5~8 打包到 0x1ff 帧。
 *        底层 HAL_CAN_AddTxMessage 非阻塞, 可在中断中使用。
 */
void DJ_Send_Frame(DJ_Motor_t* pMotor);

/**
 * @brief 反馈帧解析(CAN 接收回调中调用)
 * @note  解析 C620 反馈数据, 转换量纲并维护多圈累计(跨圈检测),
 *        最终给出输出轴多圈角度与角速度, 供位置/速度环使用。
 */
void DJ_Rx_Switch(DJ_Motor_t* pMotor, Struct_CAN_Rx_Buffer *pRxBuffer);

#endif
