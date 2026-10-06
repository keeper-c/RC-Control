/**
 * @file drv_DM_Control.c
 * @brief 达妙(DAMIAO)电机驱动实现文件
 * @version 2.0
 * @date 2026-09-04
 * @author Ckeeper
 *
 * 实现达妙系列电机(如 DM-S3519-1EC)的 CAN 总线通信、MIT/速度/位置
 * 多模式控制以及状态反馈解析。所有定点-浮点转换采用无符号偏置编码,
 * 与电机固件约定一致。
 */

#include "drv_DM_Control.h"


/* 内部函数定义 --------------------------------------------------------*/

/**
 * @brief 发送系统命令(使能/失能/清错/置零)到电机
 * @param hcan CAN 句柄指针(取自 pMotor->hcan)
 * @param ID   电机 ID(ESC_ID, 0-0x7FF),作为仲裁 ID 直接发送
 * @param cmd  命令字: 0xFC=使能 0xFD=失能 0xFE=置零 0xFB=清错
 *
 * @note 系统命令的仲裁 ID = motor_id, 不能加 0x200/0x100 等模式基址,
 *       否则电机收不到使能/失能命令,会一直停在 DISABLED 状态。
 *       数据帧前 7 字节固定 0xFF,只 D[7] 携带命令字。
 */
void DM_Motor_SendCommand(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t cmd)
{
    uint8_t Tx_Buffer[8];

    Tx_Buffer[0] = 0xFF;
    Tx_Buffer[1] = 0xFF;
    Tx_Buffer[2] = 0xFF;
    Tx_Buffer[3] = 0xFF;
    Tx_Buffer[4] = 0xFF;
    Tx_Buffer[5] = 0xFF;
    Tx_Buffer[6] = 0xFF;
    Tx_Buffer[7] = cmd;

    CAN_Send_Data(hcan, ID, Tx_Buffer, 8);
}

/* 公开函数定义 --------------------------------------------------------*/

/**
 * @brief 电机对象初始化(绑定 CAN + 配置 ID + 极限参数 + 模式 + 使能)
 * @param pMotor  电机对象指针
 * @param hcan    绑定的 CAN 句柄(后续所有发送函数都用它)
 * @param ID      电机 ID(ESC_ID),建议 0x01-0x0F
 * @param mode    控制模式(MIT/Speed/PS/OtoF)
 * @param max_pos 位置映射范围 PMAX(rad),必须与电机寄存器 21 一致
 * @param max_vel 速度映射范围 VMAX(rad/s),必须与电机寄存器 22 一致
 * @param max_t   力矩映射范围 TMAX(Nm),必须与电机寄存器 23 一致
 * @param max_kp  刚度上限(默认 500)
 * @param max_kd  阻尼上限(默认 5)
 *
 * @note 极限参数必须用达妙调试助手读出电机寄存器真实值后再传入,
 *       否则 MIT 模式的指令/反馈刻度都会失真。
 *       函数末尾会自动调用 DM_Start 使能电机。
 */
void DM_Motor_Init(DM_Motor_t *pMotor, CAN_HandleTypeDef *hcan, uint16_t ID, DM_Mode_t mode, float max_pos, float max_vel, float max_t, float max_kp, float max_kd)
{
    pMotor->hcan = hcan;
    DM_Set_ID(pMotor, ID);
    DM_Set_Max(pMotor, max_pos, max_vel, max_t, max_kp, max_kd);
    DM_Set_Mode(pMotor, mode);
    DM_Start(pMotor);
}

/**
 * @brief 使能电机(进入 ENABLED 状态,指示灯绿色常亮)
 * @param pMotor 电机对象指针
 *
 * @note 命令字 0xFC,发送后电机闭环输出力矩生效。
 *       HAL_Delay(1) 留出 CAN 帧间隔,避免连续发送挤占邮箱。
 */
void DM_Start(DM_Motor_t *pMotor)
{
    DM_Motor_SendCommand(pMotor->hcan, pMotor->ID, 0xFC); // 启动电机
    HAL_Delay(1);
}

/**
 * @brief 失能电机(进入 DISABLED 状态,指示灯红色常亮)
 * @param pMotor 电机对象指针
 *
 * @note 命令字 0xFD,失能后电机无输出力矩,但仍会持续发送反馈帧。
 */
void DM_Stop(DM_Motor_t *pMotor)
{
    DM_Motor_SendCommand(pMotor->hcan, pMotor->ID, 0xFD); // 停止电机
    HAL_Delay(1);
}

/**
 * @brief 发送速度模式控制帧(VEL mode)
 * @param pMotor 电机对象指针
 *
 * @note 帧结构: 仲裁 ID = 0x200 + motor_id, DLC=4,
 *       数据为 IEEE754 单精度浮点速度值(小端),单位 rad/s。
 *       速度值由 DM_Set_S 写入。
 */
static void DM_Send_SpeedFrame(DM_Motor_t *pMotor)
{
    CAN_Send_Data(pMotor->hcan, DM_MOTOR_Speed_ID + pMotor->ID, pMotor->TX_Frame_Speed.fu.u8, 4);
    HAL_Delay(1);
}

/**
 * @brief 发送位置-速度模式控制帧(POS_VEL mode)
 * @param pMotor 电机对象指针
 *
 * @note 帧结构: 仲裁 ID = 0x100 + motor_id, DLC=8,
 *       D[0..3]=目标位置(float,小端), D[4..7]=速度上限(float,小端)。
 *       位置和速度通过 DM_Set_PS 写入。
 */
static void DM_Send_PSFrame(DM_Motor_t *pMotor)
{
    uint8_t Tx_Buffer[8];
    Tx_Buffer[0] = pMotor->TX_Frame_PS.P_Des.u8[0];
    Tx_Buffer[1] = pMotor->TX_Frame_PS.P_Des.u8[1];
    Tx_Buffer[2] = pMotor->TX_Frame_PS.P_Des.u8[2];
    Tx_Buffer[3] = pMotor->TX_Frame_PS.P_Des.u8[3];
    Tx_Buffer[4] = pMotor->TX_Frame_PS.V_Des.u8[0];
    Tx_Buffer[5] = pMotor->TX_Frame_PS.V_Des.u8[1];
    Tx_Buffer[6] = pMotor->TX_Frame_PS.V_Des.u8[2];
    Tx_Buffer[7] = pMotor->TX_Frame_PS.V_Des.u8[3];
    CAN_Send_Data(pMotor->hcan, DM_MOTOR_PS_ID + pMotor->ID, Tx_Buffer, 8);
    HAL_Delay(1);
}

/**
 * @brief 发送 MIT 模式控制帧
 * @param pMotor 电机对象指针
 *
 * @note 帧结构(仲裁 ID = motor_id, DLC=8):
 *   D[0]      = P_Des[15:8]    位置 16bit 无符号偏置
 *   D[1]      = P_Des[7:0]
 *   D[2]      = V_Des[11:4]    速度 12bit 无符号偏置
 *   D[3]      = V_Des[3:0]<<4 | Kp[11:8]
 *   D[4]      = Kp[7:0]        刚度 12bit 无符号(0~Kp_Max)
 *   D[5]      = Kd[11:4]       阻尼 12bit 无符号(0~Kd_Max)
 *   D[6]      = Kd[3:0]<<4 | T_ff[11:8]
 *   D[7]      = T_ff[7:0]      前馈力矩 12bit 无符号偏置
 *
 * 电机的输出力矩 τ = Kp*(P_des - P) + Kd*(V_des - V) + T_ff
 * 所有定点值由 DM_Set_MIT 用 float_to_uint_damiao 转换得到,
 * 必须保证 P_Max/V_Max/T_Max/Kp_Max/Kd_Max 与电机寄存器一致。
 */
static void DM_Send_MITFrame(DM_Motor_t *pMotor)
{
    uint8_t Tx_Buffer[8];

    /* D[0..1]: 位置 P_Des 16bit, 高字节在前(大端打包) */
    Tx_Buffer[0] = (pMotor->TX_Frame_MIT.P_Des >> 8) & 0xFFU;
    Tx_Buffer[1] = pMotor->TX_Frame_MIT.P_Des & 0xFFU;

    /* D[2]: 速度 V_Des 高 8 位 V_Des[11:4] */
    Tx_Buffer[2] = (pMotor->TX_Frame_MIT.V_Des >> 4) & 0xFFU;

    /* D[3]: 速度低 4 位放高 nibble, 刚度 Kp 高 4 位放低 nibble */
    uint8_t vdes_low4 = pMotor->TX_Frame_MIT.V_Des & 0x0FU;
    uint8_t kp_high4  = (pMotor->TX_Frame_MIT.Kp >> 8) & 0x0FU;
    Tx_Buffer[3] = (vdes_low4 << 4) | kp_high4;

    /* D[4]: 刚度 Kp 低 8 位 */
    Tx_Buffer[4] = pMotor->TX_Frame_MIT.Kp & 0xFFU;

    /* D[5]: 阻尼 Kd 高 8 位 Kd[11:4] */
    Tx_Buffer[5] = (pMotor->TX_Frame_MIT.Kd >> 4) & 0xFFU;

    /* D[6]: 阻尼低 4 位放高 nibble, 力矩 T_ff 高 4 位放低 nibble */
    uint8_t kd_low4   = pMotor->TX_Frame_MIT.Kd & 0x0FU;
    uint8_t tff_high4 = (pMotor->TX_Frame_MIT.T_ff >> 8) & 0x0FU;
    Tx_Buffer[6] = (kd_low4 << 4) | tff_high4;

    /* D[7]: 力矩 T_ff 低 8 位 */
    Tx_Buffer[7] = pMotor->TX_Frame_MIT.T_ff & 0xFFU;

    CAN_Send_Data(pMotor->hcan, DM_MOTOR_MIT_ID + pMotor->ID, Tx_Buffer, 8);
    HAL_Delay(1);
}

/**
 * @brief 按当前模式发送控制帧
 * @param pMotor 电机对象指针
 *
 * @note 由 main 循环周期性调用,根据 pMotor->Mode 自动分发到对应发送函数。
 *       OtoF (FORCE_POS) 模式尚未实现,保留空分支。
 */
void DM_Send_Frame(DM_Motor_t *pMotor)
{
    switch(pMotor->Mode)
    {
        case DM_Mode_Speed:
        {
            DM_Send_SpeedFrame(pMotor);
        }
        break;

        case DM_Mode_MIT:
        {
            DM_Send_MITFrame(pMotor);
        }
        break;

        case DM_Mode_PS:
        {
            DM_Send_PSFrame(pMotor);
        }
        break;

        case DM_Mode_OtoF:
        {
            /* TODO: FORCE_POS 模式帧 (仲裁 ID = 0x300 + motor_id) */
        }
        break;

        default:
            break;
    }
}

/**
 * @brief 设置电机 ID(ESC_ID)
 * @param pMotor 电机对象指针
 * @param ID     电机 ID,作为 MIT 命令和系统命令的仲裁 ID
 */
void DM_Set_ID(DM_Motor_t *pMotor, uint8_t ID)
{
    pMotor->ID = ID;
}

/**
 * @brief 切换电机控制模式
 * @param pMotor 电机对象指针
 * @param mode   目标模式(MIT/Speed/PS/OtoF)
 */
void DM_Set_Mode(DM_Motor_t *pMotor, DM_Mode_t mode)
{
    pMotor->Mode = mode;
}

/**
 * @brief 设置速度模式目标速度
 * @param pMotor 电机对象指针
 * @param speed  目标速度(rad/s),通过 IEEE754 浮点直接发送
 */
void DM_Set_S(DM_Motor_t *pMotor, float speed)
{
    pMotor->TX_Frame_Speed.fu.f = speed;
}

/**
 * @brief 设置位置-速度模式目标
 * @param pMotor  电机对象指针
 * @param position 目标位置(rad)
 * @param speed   速度上限(rad/s)
 */
void DM_Set_PS(DM_Motor_t *pMotor, float position, float speed)
{
    pMotor->TX_Frame_PS.P_Des.f = position;
    pMotor->TX_Frame_PS.V_Des.f = speed;
}

/**
 * @brief 配置 MIT 模式的极限参数(必须与电机寄存器一致)
 * @param pMotor 电机对象指针
 * @param P_MAX  位置映射范围(rad),对应寄存器 21
 * @param V_MAX  速度映射范围(rad/s),对应寄存器 22
 * @param T_MAX  力矩映射范围(Nm),对应寄存器 23
 * @param Kp_MAX 刚度上限(默认 500)
 * @param Kd_MAX 阻尼上限(默认 5)
 *
 * @note 这五个值会作为 float_to_uint_damiao 的 x_max 使用,
 *       一旦和电机寄存器不一致,指令和反馈刻度立刻失真。
 */
void DM_Set_Max(DM_Motor_t *pMotor, float P_MAX, float V_MAX, float T_MAX, float Kp_MAX, float Kd_MAX)
{
    pMotor->TX_Frame_MIT.P_Max = P_MAX;
    pMotor->TX_Frame_MIT.T_Max = T_MAX;
    pMotor->TX_Frame_MIT.V_Max = V_MAX;
    pMotor->TX_Frame_MIT.Kp_Max = Kp_MAX;
    pMotor->TX_Frame_MIT.Kd_Max = Kd_MAX;
}

/**
 * @brief 设置 MIT 模式控制目标
 * @param pMotor   电机对象指针
 * @param position 目标位置(rad), 范围 [-P_Max, +P_Max]
 * @param speed    目标速度(rad/s), 范围 [-V_Max, +V_Max]
 * @param kp       位置刚度, 范围 [0, Kp_Max]
 * @param kd       速度阻尼, 范围 [0, Kd_Max]
 * @param t_ff     前馈力矩(Nm), 范围 [-T_Max, +T_Max]
 *
 * @note 位置/速度/力矩采用无符号偏置编码([-max,+max] -> [0, 2^N-1], 中点=0);
 *       Kp/Kd 采用无符号编码([0,max] -> [0, 2^N-1])。
 *       传入值会先做饱和再转换,避免越界。
 */
void DM_Set_MIT(DM_Motor_t *pMotor, float position, float speed, float kp, float kd, float t_ff)
{
    pMotor->TX_Frame_MIT.P_Des  = float_to_uint_damiao(position, -pMotor->TX_Frame_MIT.P_Max, pMotor->TX_Frame_MIT.P_Max, 16);
    pMotor->TX_Frame_MIT.V_Des  = float_to_uint_damiao(speed,    -pMotor->TX_Frame_MIT.V_Max, pMotor->TX_Frame_MIT.V_Max, 12);
    pMotor->TX_Frame_MIT.Kp     = float_to_uint_damiao(kp,       0.0f,                       pMotor->TX_Frame_MIT.Kp_Max, 12);
    pMotor->TX_Frame_MIT.Kd     = float_to_uint_damiao(kd,       0.0f,                       pMotor->TX_Frame_MIT.Kd_Max, 12);
    pMotor->TX_Frame_MIT.T_ff   = float_to_uint_damiao(t_ff,     -pMotor->TX_Frame_MIT.T_Max, pMotor->TX_Frame_MIT.T_Max, 12);
}


/**
 * @brief 解析电机反馈帧,填入 RX_Frame
 * @param pMotor    电机对象指针
 * @param pRxBuffer CAN 接收缓冲区指针
 *
 * @note 反馈帧结构(仲裁 ID = MST_ID, DLC=8):
 *   D[0]      = ERR[7:4] | motor_id[3:0]
 *   D[1..2]   = 位置 16bit 无符号偏置
 *   D[3]      = 速度高 8 位 V[11:4]
 *   D[4]      = V[3:0] | T[11:8]
 *   D[5]      = 力矩低 8 位 T[7:0]
 *   D[6]      = MOS 管温度(℃)
 *   D[7]      = 转子温度(℃)
 *
 * 关键: 所有数值字段都是无符号偏置编码, 不要做有符号位扩展,
 *       否则静止(raw=2047)会被误读成最大值。
 */
void DM_RX_Switch(DM_Motor_t *pMotor, Struct_CAN_Rx_Buffer *pRxBuffer)
{
    /* 仲裁 ID 即主机 ID(MST_ID); D[0] 高 4 位是错误码, 低 4 位是电机 ID */
    pMotor->RX_Frame.MST_ID  = pRxBuffer->Header.StdId;
    pMotor->RX_Frame.ID      = pRxBuffer->Data[0] & 0x0FU;
    pMotor->RX_Frame.ERR     = (pRxBuffer->Data[0] >> 4U) & 0x0FU;

    /* 反馈帧是无符号偏置编码, 不需要符号扩展 */
    uint16_t pos_raw = ((uint16_t)pRxBuffer->Data[1] << 8U) | pRxBuffer->Data[2];
    uint16_t vel_raw = ((uint16_t)pRxBuffer->Data[3] << 4U) | ((pRxBuffer->Data[4] >> 4U) & 0x0FU);
    uint16_t t_raw   = (((uint16_t)pRxBuffer->Data[4] & 0x0FU) << 8U) | pRxBuffer->Data[5];

    /* 定点 -> 浮点, 用与编码端相同的极限值做反变换, 直接存入 RX_Frame 的 float 成员 */
    pMotor->RX_Frame.POS = uint_to_float_damiao(pos_raw, -pMotor->TX_Frame_MIT.P_Max, pMotor->TX_Frame_MIT.P_Max, 16);
    pMotor->RX_Frame.VEL = uint_to_float_damiao(vel_raw, -pMotor->TX_Frame_MIT.V_Max, pMotor->TX_Frame_MIT.V_Max, 12);
    pMotor->RX_Frame.T   = uint_to_float_damiao(t_raw,   -pMotor->TX_Frame_MIT.T_Max, pMotor->TX_Frame_MIT.T_Max, 12);

    /* 温度字段是原始字节数据, 单位 ℃, 直接复制 */
    pMotor->RX_Frame.T_MOS   = pRxBuffer->Data[6];
    pMotor->RX_Frame.T_Rotor = pRxBuffer->Data[7];
}
