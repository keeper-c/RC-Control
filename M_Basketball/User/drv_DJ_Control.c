#include "drv_DJ_Control.h"

uint8_t DJ_Send_Data_0x200[DJ_DATA_FRAME_SIZE];
uint8_t DJ_Send_Data_0x1ff[DJ_DATA_FRAME_SIZE];


void DJ_Motor_Init(DJ_Motor_t* pMotor, CAN_HandleTypeDef *hcan, uint8_t ID, DJ_Motor_Mode_t mode, float reduction_ratio, uint8_t* data_frame)
{
    pMotor->hcan = hcan;
    pMotor->ID = ID;
    pMotor->data = data_frame;
    pMotor->mode = mode;
    pMotor->RX_Frame.reduction_ratio = reduction_ratio;
    pMotor->RX_Frame.last_ecd = 0;
    pMotor->RX_Frame.round_cnt = 0;
    pMotor->RX_Frame.total_rotor_rad = 0.0f;
    pMotor->RX_Frame.total_out_rad = 0.0f;
    pMotor->RX_Frame.omega_out_rad_s = 0.0f;

    PID_Init(&pMotor->speed_pid, PID_POSITION, KP, KI, KD, DJI_INTEGRAL_MAX, DJI_CURRENT_MAX);
    PID_Init(&pMotor->position_pid, PID_POSITION, PKP, PKI, PKD, DJI_POSITION_INTEGRAL_MAX, DJI_POSITION_OUT_MAX);
    //pMotor->position_pid.deadband = 0.02f;  /* 位置环死区 */
}

/**
 * @brief 设置目标(速度模式直给速度环; 位置模式启动抛物线轨迹生成器)
 * @note  位置模式: 不直接赋 target, 而是记录 起点=当前反馈位置、终点=target、
 *        T=DJ_TRAJ_TIME_MS, 在 DJ_Motor_Control 里按 τ=t/T 抛物线插值更新目标。
 *        抛物线规划: 前半段位置=2τ²、后半段=1-2(1-τ)², 速度三角形(起止=0),
 *        加速度恒定 ±4Δ/T², 峰速 2Δ/T(中点), 消除阶跃冲击, 改善到位振荡。
 */
void DJ_Set_Target(DJ_Motor_t* pMotor, float target)
{
    if(pMotor->mode == DJ_MOTOR_MODE_SPEED)
    {
        PID_Set_Target(&pMotor->speed_pid, target);
    }
    else if(pMotor->mode == DJ_MOTOR_MODE_POSITION)
    {
        pMotor->traj_active = 1u;
        pMotor->traj_t  = 0.0f;
        pMotor->traj_T  = DJ_TRAJ_TIME_MS;
        pMotor->traj_p0 = pMotor->RX_Frame.total_out_rad;  /* 起点=当前反馈 */
        pMotor->traj_p1 = target;                          /* 终点 */
    }
}

/**
 * @brief 直接设置电流(绕过 PID, 调试用)
 */
void DJ_Set_Current(DJ_Motor_t* pMotor, int16_t current)
{
    pMotor->current = current;
}

/**
 * @brief 发送电流指令帧
 * @note 由 TIM1 1ms 中断调用(与速度环同周期), 满足 C620 命令看门狗要求。
 *       内部不能用 HAL_Delay: SysTick 优先级低于 TIM1, 在中断里调用会死锁。
 *       CAN_Send_Data 底层是 HAL_CAN_AddTxMessage 邮箱写入, 非阻塞, 可在中断中使用。
 */
void DJ_Send_Frame(DJ_Motor_t* pMotor)
{
    if(pMotor->ID <= 0x04)
    {
        uint8_t index = (pMotor->ID - 1) * 2;
        pMotor->data[index] = (pMotor->current >> 8) & 0xFF;
        pMotor->data[index + 1] = (pMotor->current) & 0xFF;
        CAN_Send_Data(pMotor->hcan, DJ_MOTOR_ADDR_0x200, pMotor->data, DJ_DATA_FRAME_SIZE);
    }
    else if(pMotor->ID <= 0x08)
    {
        uint8_t index = (pMotor->ID - 5) * 2;
        pMotor->data[index] = (pMotor->current >> 8) & 0xFF;
        pMotor->data[index + 1] = (pMotor->current) & 0xFF;
        CAN_Send_Data(pMotor->hcan, DJ_MOTOR_ADDR_0x1ff, pMotor->data, DJ_DATA_FRAME_SIZE);
    }
}

void DJ_Rx_Switch(DJ_Motor_t* pMotor, Struct_CAN_Rx_Buffer *pRxBuffer)
{
    pMotor->RX_Frame.RX_ID = pRxBuffer->Header.StdId;
    pMotor->RX_Frame.P_Des = (int16_t)(pRxBuffer->Data[0] << 8 | pRxBuffer->Data[1]);
    pMotor->RX_Frame.V_Des = (int16_t)(pRxBuffer->Data[2] << 8 | pRxBuffer->Data[3]);
    pMotor->RX_Frame.Current = (int16_t)(pRxBuffer->Data[4] << 8 | pRxBuffer->Data[5]);
    pMotor->RX_Frame.Temperature = pRxBuffer->Data[6];

    // --------转子侧（电机内部高速轴）--------
    pMotor->RX_Frame.mech_rad = pMotor->RX_Frame.P_Des * DJI_ECD_TO_RAD;//转子单圈机械角度 0~2π rad
    pMotor->RX_Frame.omega_rotor_rad_s = pMotor->RX_Frame.V_Des * RPM_TO_RAD_S;//转子角速度 rad/s

    // 多圈转子累计
    int16_t curr_ecd = pMotor->RX_Frame.P_Des;//转子单圈编码器 0~8191
    int16_t delta_ecd = curr_ecd - pMotor->RX_Frame.last_ecd;//转子单圈编码器变化量

    if(delta_ecd >  ECD_MAX/2)
    {
        pMotor->RX_Frame.round_cnt--;
    }
    else if(delta_ecd < -ECD_MAX/2)
    {
        pMotor->RX_Frame.round_cnt++;
    }

    int32_t total_ecd = pMotor->RX_Frame.round_cnt * 8192 + curr_ecd;//转子多圈总编码器值
    pMotor->RX_Frame.total_rotor_rad = total_ecd * DJI_ECD_TO_RAD;//转子多圈总弧度
    pMotor->RX_Frame.last_ecd = curr_ecd;//上一帧编码器值

    // --------输出轴（经过减速箱，低速轴）--------
    // 减速：转子转 reduction_ratio 圈 → 输出轴转1圈，所以 / 减速比
    pMotor->RX_Frame.total_out_rad  = pMotor->RX_Frame.total_rotor_rad / pMotor->RX_Frame.reduction_ratio;
    pMotor->RX_Frame.omega_out_rad_s = pMotor->RX_Frame.omega_rotor_rad_s / pMotor->RX_Frame.reduction_ratio;

}

/**
 * @brief 电机控制总入口(在 TIM1 1ms 中断中调用), 运算后自动发送电流帧
 * @note  速度模式: 目标由 DJ_Set_Target_S 给定, 速度环直接输出电流;
 *        位置模式: 串级控制 ——
 *          位置环(外环, 反馈 total_out_rad 多圈角度) -> 输出目标速度(限幅 ±DJI_POSITION_OUT_MAX)
 *          速度环(内环, 反馈 omega_out_rad_s)       -> 输出电流
 *        外环输出限幅即内环速度上限, 保证内环始终有能力跟随外环。
 */
void DJ_Motor_Control(DJ_Motor_t* pMotor)
{
    if(pMotor->mode == DJ_MOTOR_MODE_SPEED)
    {
        DJ_Set_Current(pMotor, (int16_t)PID_Calculate(&pMotor->speed_pid, pMotor->RX_Frame.omega_out_rad_s));
    }
    else if(pMotor->mode == DJ_MOTOR_MODE_POSITION)
    {
        /* 抛物线轨迹生成器: 每拍按 τ=t/T 更新位置环目标
           前半 τ<0.5: p = p0 + Δ·2τ²          (加速段, 速度线性升)
           后半 τ≥0.5: p = p0 + Δ·(1-2(1-τ)²)  (减速段, 速度线性降)
           t≥T: 轨迹结束, 目标锁定终点 */
        if(pMotor->traj_active)
        {
            pMotor->traj_t += 1.0f;   /* 1ms/拍 */
            float tau = pMotor->traj_t / pMotor->traj_T;
            if(tau >= 1.0f)
            {
                tau = 1.0f;
                pMotor->traj_active = 0u;
            }
            float delta = pMotor->traj_p1 - pMotor->traj_p0;
            float s = (tau < 0.5f) ? (2.0f * tau * tau)
                                   : (1.0f - 2.0f * (1.0f - tau) * (1.0f - tau));
            PID_Set_Target(&pMotor->position_pid, pMotor->traj_p0 + delta * s);
        }
        /* 外环: 位置误差 -> 目标速度 */
        float target_speed = PID_Calculate(&pMotor->position_pid, pMotor->RX_Frame.total_out_rad);
        /* 内环: 目标速度 -> 电流 */
        PID_Set_Target(&pMotor->speed_pid, target_speed);
        DJ_Set_Current(pMotor, (int16_t)PID_Calculate(&pMotor->speed_pid, pMotor->RX_Frame.omega_out_rad_s));
    }
    DJ_Send_Frame(pMotor);
}
