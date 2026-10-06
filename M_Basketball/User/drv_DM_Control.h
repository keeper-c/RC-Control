/**
 * @file drv_DM_Control.h
 * @brief 达妙(DAMIAO)电机驱动头文件
 * @version 2.0
 * @date 2026-09-04
 * @author Ckeeper
 *
 * 定义达妙系列电机(如 DM-S3519-1EC)通信所需的数据结构、宏定义、
 * 函数声明以及定点-浮点转换的内联函数。所有定点数据采用无符号偏置编码,
 * 与电机固件约定一致。
 */

#ifndef __DM_CONTROL_H__
#define __DM_CONTROL_H__

/* ==================== 使用示例 ====================
   以单电机(DM-S3519-1EC, ID=1, CAN1, 速度模式)为例:

   1. 定义电机对象(全局)
        DM_Motor_t p_DMMotor;

   2. CAN 接收回调中解析反馈(反馈帧 ID = 主机ID/MST_ID, 默认 0x11 + 电机序号)
        回调里: if(StdId == 0x11)  DM_RX_Switch(&p_DMMotor, &rx_buf);

   3. 初始化并使能(main 外设初始化之后; PMAX/VMAX/TMAX 按电机调试软件寄存器值填)
        DM_Motor_Init(&p_DMMotor, &hcan1, 0x01, DM_Mode_Speed,
                      DM_MIT_P_MAX, DM_MIT_V_MAX, DM_MIT_T_MAX,
                      DM_MIT_KP_MAX, DM_MIT_KD_MAX);   // 内部已调用 DM_Start 使能

   4. 周期性设置目标 + 发送(建议 1ms~10ms, 定时器中断里):
        void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
        {
            if(htim->Instance == TIM1)
            {
                DM_Set_S(&p_DMMotor, 5.0f);     // 速度模式: 5 rad/s
                // DM_Set_PS(&p_DMMotor, 3.14f, 10.0f); // PS模式: 位置3.14rad, 限速10rad/s
                // DM_Set_MIT(&p_DMMotor, 0, 0, 10, 1, 0); // MIT模式: 刚度位置控制
                DM_Send_Frame(&p_DMMotor);      // 按当前模式自动选择帧格式发送
            }
        }

   5. 读取反馈(任意时刻)
        float pos = p_DMMotor.RX_Frame.POS;    // 实际位置 rad
        float vel = p_DMMotor.RX_Frame.VEL;    // 实际速度 rad/s
        uint8_t err = p_DMMotor.RX_Frame.ERR;  // 0=失能 1=使能 8+E=故障

   其他: DM_Stop 失能 / DM_Start 重新使能 / DM_Set_Mode 切换模式(切后需重新 DM_Start);
   多电机各建一个 DM_Motor_t, 反馈 ID 各自对应。
   =================================================== */

#include "main.h"
#include "drv_can.h"

/* 宏定义 ----------------------------------------------------------------*/

/**
 * @brief 各控制模式帧的仲裁 ID 基址
 * @note 实际仲裁 ID = 基址 + motor_id (ESC_ID)
 *       系统命令(使能/失能/清错/置零)直接用 motor_id, 不走这些基址
 */
#define DM_MOTOR_MIT_ID    0x000   /**< MIT 模式命令帧基址 */
#define DM_MOTOR_PS_ID     0x100   /**< 位置-速度模式命令帧基址 */
#define DM_MOTOR_Speed_ID  0x200   /**< 速度模式命令帧基址 */

/**
 * @brief MIT 模式极限参数默认值
 * @note 这些是宏默认值,实际使用时应优先用 DM_Motor_Init 传入从电机寄存器
 *       读出的真实值。各参数对应电机寄存器:
 *         DM_MIT_P_MAX  -> 寄存器 21 PMAX (rad)
 *         DM_MIT_V_MAX  -> 寄存器 22 VMAX (rad/s)
 *         DM_MIT_T_MAX  -> 寄存器 23 TMAX (Nm)
 *         DM_MIT_KP_MAX -> 刚度上限(电机约定默认 500)
 *         DM_MIT_KD_MAX -> 阻尼上限(电机约定默认 5)
 */
#define DM_MIT_P_MAX   12.5f    /**< 位置映射范围(rad), ≈4π */
#define DM_MIT_V_MAX   200.0f     /**< 速度映射范围(rad/s), 需按电机型号核对 */
#define DM_MIT_T_MAX   10.0f      /**< 力矩映射范围(Nm), 需按电机型号核对 */
#define DM_MIT_KP_MAX  500.0f     /**< 刚度上限 */
#define DM_MIT_KD_MAX  5.0f       /**< 阻尼上限 */

/* 类型定义 --------------------------------------------------------------*/

/**
 * @brief 浮点-字节联合体
 *
 * 用于速度/位置-速度模式下,直接把 float 拆成 4 字节通过 CAN 发送(IEEE754 小端)。
 */
typedef union
{
    float f;        /**< 浮点形式 */
    uint8_t u8[4];  /**< 字节形式, u8[0] 为最低字节 */
}DM_Float_t;

/**
 * @brief 电机反馈帧解析结果
 *
 * 对应电机反馈帧(仲裁 ID = MST_ID, DLC=8)解析后的物理量。
 * 所有定点字段已通过 uint_to_float_damiao 转换为浮点物理量。
 */
typedef struct
{
    uint16_t MST_ID;     /**< 主机 ID(反馈帧的仲裁 ID) */
    uint8_t  ID;        /**< 电机 ID, 来自 D[0] 低 4 位 */
    uint8_t  ERR;       /**< 错误码, 来自 D[0] 高 4 位 (0=DISABLED 1=ENABLED 8-E=故障) */
    float    POS;       /**< 实际位置(rad), 16bit 无符号偏置解码 */
    float    VEL;       /**< 实际速度(rad/s), 12bit 无符号偏置解码 */
    float    T;         /**< 实际力矩(Nm), 12bit 无符号偏置解码 */
    uint8_t  T_MOS;     /**< MOS 管温度(℃) */
    uint8_t  T_Rotor;   /**< 转子温度(℃) */
}DM_RX_Frame_t;

/**
 * @brief 速度模式(VEL mode)发送帧
 * @note 帧结构: 仲裁 ID = 0x200 + motor_id, DLC=4
 *       4 字节 IEEE754 float 速度值, 小端
 */
typedef struct
{
    DM_Float_t fu;      /**< 目标速度(rad/s) */
}DM_TX_SpeedFrame_t;

/**
 * @brief 位置-速度模式(POS_VEL mode)发送帧
 * @note 帧结构: 仲裁 ID = 0x100 + motor_id, DLC=8
 *       D[0..3]=目标位置(float, 小端), D[4..7]=速度上限(float, 小端)
 */
typedef struct
{
    DM_Float_t P_Des;   /**< 目标位置(rad) */
    DM_Float_t V_Des;   /**< 速度上限(rad/s) */
}DM_TX_PSFrame_t;

/**
 * @brief MIT 模式发送帧
 * @note 帧结构: 仲裁 ID = motor_id, DLC=8
 *       所有字段为无符号偏置编码(位置/速度/力矩)或无符号编码(Kp/Kd),
 *       位打包见 drv_DM_Control.c 中 DM_Send_MITFrame 的注释。
 */
typedef struct
{
    /* 以下 5 个字段为发送前已转换好的定点值(无符号) */
    uint16_t P_Des;     /**< 目标位置, 16bit, 范围 [0, 65535], 中点 32767 = 0 */
    uint16_t V_Des;     /**< 目标速度, 12bit, 范围 [0, 4095], 中点 2047 = 0 */
    uint16_t Kp;        /**< 位置刚度, 12bit, 范围 [0, 4095], 0~Kp_Max 线性映射 */
    uint16_t Kd;        /**< 速度阻尼, 12bit, 范围 [0, 4095], 0~Kd_Max 线性映射 */
    uint16_t T_ff;      /**< 前馈力矩, 12bit, 范围 [0, 4095], 中点 2047 = 0 */

    /* 以下 5 个字段为极限参数, 必须与电机寄存器 21/22/23 一致 */
    float P_Max;        /**< 位置映射范围(rad), 对应寄存器 21 */
    float T_Max;        /**< 力矩映射范围(Nm), 对应寄存器 23 */
    float V_Max;        /**< 速度映射范围(rad/s), 对应寄存器 22 */
    float Kp_Max;       /**< 刚度上限(默认 500) */
    float Kd_Max;       /**< 阻尼上限(默认 5) */
}DM_TX_MITFrame_t;

/**
 * @brief 电机控制模式枚举
 */
typedef enum
{
    DM_Mode_Speed = 0,  /**< 速度模式(VEL), 仲裁 ID = 0x200 + motor_id */
    DM_Mode_MIT,        /**< MIT 模式, 仲裁 ID = motor_id */
    DM_Mode_PS,         /**< 位置-速度模式(POS_VEL), 仲裁 ID = 0x100 + motor_id */
    DM_Mode_OtoF,       /**< 力位混合模式(FORCE_POS), 仲裁 ID = 0x300 + motor_id */
}DM_Mode_t;

/**
 * @brief 电机对象(每个电机一个实例)
 *
 * 包含绑定的 CAN 句柄、ID、模式、反馈帧缓存和三种发送帧缓存。
 * 使用者按当前 Mode 调用对应的 DM_Set_* 填充发送帧,再调用 DM_Send_Frame 发送,
 * 内部所有 CAN 发送都用 pMotor->hcan, 不再硬编码 &hcan1。
 */
typedef struct
{
    CAN_HandleTypeDef    *hcan;          /**< 绑定的 CAN 句柄,由 DM_Motor_Init 设置,所有发送函数都用它 */
    uint8_t              ID;             /**< 电机 ID(ESC_ID), 作为命令帧仲裁 ID */
    DM_RX_Frame_t        RX_Frame;       /**< 反馈帧缓存 */
    DM_TX_SpeedFrame_t   TX_Frame_Speed; /**< 速度模式发送帧缓存 */
    DM_TX_PSFrame_t      TX_Frame_PS;    /**< 位置-速度模式发送帧缓存 */
    DM_TX_MITFrame_t     TX_Frame_MIT;   /**< MIT 模式发送帧缓存 */
    DM_Mode_t            Mode;           /**< 当前控制模式 */
}DM_Motor_t;

/* 函数声明 --------------------------------------------------------------*/

/**
 * @brief 电机对象初始化(绑定 CAN + 配置 ID + 极限参数 + 模式 + 使能)
 * @param pMotor  电机对象指针
 * @param hcan    绑定的 CAN 句柄(后续所有发送函数都用它)
 * @param ID      电机 ID(ESC_ID)
 * @param mode    控制模式
 * @param max_pos 位置映射范围 PMAX(rad)
 * @param max_vel 速度映射范围 VMAX(rad/s)
 * @param max_t   力矩映射范围 TMAX(Nm)
 * @param max_kp  刚度上限(默认 500)
 * @param max_kd  阻尼上限(默认 5)
 */
void DM_Motor_Init(DM_Motor_t *pMotor, CAN_HandleTypeDef *hcan, uint16_t ID, DM_Mode_t mode, float max_pos, float max_vel, float max_t, float max_kp, float max_kd);

/**
 * @brief 使能电机(命令字 0xFC)
 */
void DM_Start(DM_Motor_t *pMotor);

/**
 * @brief 失能电机(命令字 0xFD)
 */
void DM_Stop(DM_Motor_t *pMotor);

/**
 * @brief 按当前模式发送控制帧(由 main 循环周期性调用)
 */
void DM_Send_Frame(DM_Motor_t *pMotor);

/**
 * @brief 解析电机反馈帧, 填入 pMotor->RX_Frame
 * @param pRxBuffer CAN 接收缓冲区指针
 */
void DM_RX_Switch(DM_Motor_t *pMotor, Struct_CAN_Rx_Buffer *pRxBuffer);

/**
 * @brief 切换电机控制模式
 */
void DM_Set_Mode(DM_Motor_t *pMotor, DM_Mode_t mode);

/**
 * @brief 设置速度模式目标速度
 * @param speed 目标速度(rad/s)
 */
void DM_Set_S(DM_Motor_t *pMotor, float speed);

/**
 * @brief 设置位置-速度模式目标
 * @param position 目标位置(rad)
 * @param speed    速度上限(rad/s)
 */
void DM_Set_PS(DM_Motor_t *pMotor, float position, float speed);

/**
 * @brief 设置电机 ID(ESC_ID)
 */
void DM_Set_ID(DM_Motor_t *pMotor, uint8_t ID);

/**
 * @brief 配置 MIT 模式极限参数(必须与电机寄存器一致)
 */
void DM_Set_Max(DM_Motor_t *pMotor, float P_MAX, float V_MAX, float T_MAX, float Kp_MAX, float Kd_MAX);

/**
 * @brief 设置 MIT 模式控制目标
 * @param position 目标位置(rad)
 * @param speed    目标速度(rad/s)
 * @param kp       位置刚度
 * @param kd       速度阻尼
 * @param t_ff     前馈力矩(Nm)
 */
void DM_Set_MIT(DM_Motor_t *pMotor, float position, float speed, float kp, float kd, float t_ff);

void DM_Motor_SendCommand(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t cmd);


/* 内联函数 --------------------------------------------------------------*/

/**
 * @brief 浮点 -> 无符号偏置定点(达妙电机编码方式)
 * @param val   待转换的浮点值
 * @param x_min 物理量下限(位置/速度/力矩为 -MAX, Kp/Kd 为 0)
 * @param x_max 物理量上限(+MAX)
 * @param bits  定点位宽(位置=16, 速度/力矩/Kp/Kd=12)
 * @return      无符号定点值, 范围 [0, 2^bits - 1]
 *
 * @note 编码规则:
 *         - 位置/速度/力矩(bidirectional): [x_min, x_max] -> [0, 2^N-1], 中点对应 0
 *         - Kp/Kd (unipolar):              [0, x_max]      -> [0, 2^N-1]
 *       传入值会先做饱和再转换, 避免越界。
 */
static inline uint16_t float_to_uint_damiao(float val, float x_min, float x_max, uint16_t bits)
{
    if(val >  x_max) val =  x_max;
    if(val <  x_min) val =  x_min;
    float span = x_max - x_min;
    float data_norm = (val - x_min) / span;
    return (uint16_t)(data_norm * (float)((1U << bits) - 1));
}

/**
 * @brief 无符号偏置定点 -> 浮点(达妙电机解码方式, float_to_uint_damiao 的逆运算)
 * @param raw   无符号定点值, 范围 [0, 2^bits - 1]
 * @param x_min 物理量下限
 * @param x_max 物理量上限
 * @param bits  定点位宽
 * @return      解码后的物理量(float)
 *
 * @note 解码时 x_min/x_max 必须与编码时一致, 否则刻度失真。
 */
static inline float uint_to_float_damiao(uint16_t raw, float x_min, float x_max, uint16_t bits)
{
    float span = x_max - x_min;
    float data_norm = (float)raw / (float)((1U << bits) - 1);
    return data_norm * span + x_min;
}

#endif
