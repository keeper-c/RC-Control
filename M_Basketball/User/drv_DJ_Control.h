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
#define DJI_INTEGRAL_MAX 200.0f     /* 速度环积分限幅(raw) */
#define KP   200.0f                  /* 速度环比例: raw/(rad/s) */
#define KI   0.3f                    /* 速度环积分: raw/(rad/s·tick) */
#define KD   1.5f                   /* 速度环微分: raw/(rad/s) */

/* 位置环(串级外环): 反馈 rad(输出轴多圈角度), 输出目标速度 rad/s 给速度环 */
#define PKP  150.0f                   /* 位置环比例: (rad/s)/rad, 到位附近推力 */
#define PKI  0.005f                     /* 位置环积分: 串级外环用纯 P, 内环积分消稳态差; 加了容易振荡 */
#define PKD  30.0f                     /* 位置环微分: 接近目标时刹车, 抑制冲动 */
#define DJI_POSITION_INTEGRAL_MAX 200.0f /* 位置环积分限幅(禁用也保留字段) */
#define DJI_POSITION_OUT_MAX 40.0f   /* 位置环输出限幅 = 内环最大目标速度(rad/s) */
#define DJ_TRAJ_TIME_MS  800.0f      /* 抛物线轨迹时间(ms) */



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
    PID_t speed_pid;   /**< 速度环 PID 控制器 */
    PID_t position_pid; /**< 位置式环 PID 控制器 */

    /* 抛物线轨迹生成器(仅位置模式有效, DJ_Set_Target 启动) */
    uint8_t traj_active;  /**< 轨迹进行中标志 */
    float    traj_t;      /**< 已运行时间(ms) */
    float    traj_T;      /**< 总过渡时长(ms) */
    float    traj_p0;     /**< 起点位置(rad, 输出轴) */
    float    traj_p1;     /**< 终点位置(rad, 输出轴) */
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
 * @brief 设置目标(速度模式直给速度环; 位置模式启动抛物线轨迹生成器)
 * @param target 速度模式: 目标速度 rad/s; 位置模式: 目标位置 rad
 * @note  位置模式不直接赋 PID.target, 而是记录起终点, 在 DJ_Motor_Control 里按
 *        τ=t/T 抛物线插值逐步更新目标(速度三角形, 起止速度=0), 消除阶跃冲击。
 */
void DJ_Set_Target(DJ_Motor_t* pMotor, float target);

/**
 * @brief 电机控制总入口(在 TIM1 1ms 中断中调用)
 * @note  速度模式: 速度环 PID -> 电流;
 *        位置模式: 抛物线轨迹更新目标 -> 位置环(外环) -> 速度环(内环) -> 电流。
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
