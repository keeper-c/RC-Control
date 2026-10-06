#include "drv_DJ_Control.h"
#include <math.h>

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
    pMotor->RX_Frame.omega_filt = 0.0f;

    PID_Init(&pMotor->speed_pid, PID_POSITION, KP, KI, KD, DJI_INTEGRAL_MAX, DJI_CURRENT_MAX);
    PID_Init(&pMotor->position_pid, PID_POSITION, PKP, PKI, PKD, DJI_POSITION_INTEGRAL_MAX, DJI_POSITION_OUT_MAX);

    /* PID 增强(参考 Motor.c 四件套): 位置环 微分先行+低通 + 积分分离 */
    pMotor->position_pid.sep   = DJI_POS_SEP;
    pMotor->position_pid.alpha = DJI_POS_ALPHA;

    /* 在线梯形速度规划状态: 首帧反馈到来后同步轨迹基准(plan_sync) */
    pMotor->plan_sync  = 0u;
    pMotor->plan_done  = 1u;
    pMotor->tgt_pos    = 0.0f;
    pMotor->pos_ref    = 0.0f;
    pMotor->v_ref      = 0.0f;
    pMotor->plan_v_max = DJI_TRAJ_V_MAX;
    //pMotor->position_pid.deadband = 0.02f;  /* 位置环死区 */
    pMotor->position_pid.deadband = 0.001f;  /* 小扰动免疫: |err|<0.005rad(0.29°)时位置环不响应 */
}

/**
 * @brief 设置目标(统一双参接口: 位置 + 速度)
 * @param pos   位置模式: 目标位置 rad(输出轴多圈); 速度模式: 忽略
 * @param speed 速度模式: 目标速度 rad/s(输出轴); 位置模式: 本次移动匀速上限 rad/s
 * @note  速度模式: speed 直给速度环, pos 被忽略;
 *        位置模式: 记录终点(tgt_pos)并以 speed 为匀速上限启动在线梯形规划
 *        (plan_done=0), DJ_Motor_Control 每拍按剩余距离弧形接近,
 *        到位自动交棒给位置环精调。speed 钳位到 [0.05, DJI_TRAJ_V_MAX],
 *        减速包络与加减速度仍由 DJI_DEC_ENV_K/DJI_TRAJ_A_MAX 决定。
 */
void DJ_Set_Target(DJ_Motor_t* pMotor, float pos, float speed)
{
    if(pMotor->mode == DJ_MOTOR_MODE_SPEED)
    {
        PID_Set_Target(&pMotor->speed_pid, speed);   /* 速度模式: 忽略位置 */
    }
    else if(pMotor->mode == DJ_MOTOR_MODE_POSITION)
    {
        if(speed > DJI_TRAJ_V_MAX) speed = DJI_TRAJ_V_MAX;
        if(speed < 0.05f)          speed = 0.05f;   /* 防呆: 速度过小卡死在规划段 */

        pMotor->tgt_pos    = pos;
        pMotor->plan_v_max = speed;
        pMotor->plan_done  = 0u;   /* 触发新规划 */
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

    /* 速度反馈一阶低通(速度环反馈用): 滤波结果喂 PID, 原始值保留 */
    pMotor->RX_Frame.omega_filt = DJI_SPD_FILT * pMotor->RX_Frame.omega_out_rad_s
                                + (1.0f - DJI_SPD_FILT) * pMotor->RX_Frame.omega_filt;

    /* 首帧反馈: 规划轨迹基准同步到当前实际位置(防止上电时位置环冲向 0) */
    if(!pMotor->plan_sync)
    {
        pMotor->plan_sync = 1u;
        pMotor->pos_ref   = pMotor->RX_Frame.total_out_rad;
        pMotor->tgt_pos   = pMotor->RX_Frame.total_out_rad;
        pMotor->v_ref     = 0.0f;
    }
}

/**
 * @brief 电机控制总入口(在 TIM1 1ms 中断中调用), 运算后自动发送电流帧
 * @note  速度模式: 速度环 PID(反馈用低通后 omega_filt) -> 电流;
 *        位置模式: 在线梯形规划 -> 位置环 PID + 规划速度前馈 -> 速度环 PID -> 电流。
 *        规划器: v_allow = min(V_MAX, K·√(2·A·|s_rem|)), v_ref 斜坡逼近,
 *        pos_ref 数值积分; 到位交棒看实际反馈(误差窗+速度窗), 轨迹钉死目标。
 *        位置环输出为速度修正量, 与前馈合成速度目标, 前馈不经过位置环限幅。
 */
void DJ_Motor_Control(DJ_Motor_t* pMotor)
{
    if(pMotor->mode == DJ_MOTOR_MODE_SPEED)
    {
        /* 目标由 DJ_Set_Target 给定; 反馈用低通后的 omega_filt */
        DJ_Set_Current(pMotor, (int16_t)PID_Calculate(&pMotor->speed_pid, pMotor->RX_Frame.omega_filt));
    }
    else if(pMotor->mode == DJ_MOTOR_MODE_POSITION)
    {
        /* ---- ① 在线梯形速度规划 ---- */
        if(!pMotor->plan_done)
        {
            float s_rem = pMotor->tgt_pos - pMotor->pos_ref;         /* 剩余距离(规划量) */
            float v_dec = DJI_DEC_ENV_K * sqrtf(2.0f * DJI_TRAJ_A_MAX * fabsf(s_rem));
            float v_cmd = (s_rem > 0.0f) ? v_dec : -v_dec;
            float dv_max = DJI_TRAJ_A_MAX * DJI_CTRL_DT;

            if(v_cmd >  pMotor->plan_v_max) v_cmd =  pMotor->plan_v_max;
            if(v_cmd < -pMotor->plan_v_max) v_cmd = -pMotor->plan_v_max;

            /* v_ref 朝 v_cmd 斜坡逼近, 每拍最多变 A·dt */
            if(pMotor->v_ref < v_cmd)
            {
                pMotor->v_ref += dv_max;
                if(pMotor->v_ref > v_cmd) pMotor->v_ref = v_cmd;
            }
            else if(pMotor->v_ref > v_cmd)
            {
                pMotor->v_ref -= dv_max;
                if(pMotor->v_ref < v_cmd) pMotor->v_ref = v_cmd;
            }
            pMotor->pos_ref += pMotor->v_ref * DJI_CTRL_DT;          /* 位置轨迹=速度积分 */
        }

        /* ---- ② 到位交棒判定: 看实际反馈, 不看规划量 ---- */
        if(fabsf(pMotor->tgt_pos - pMotor->RX_Frame.total_out_rad) < DJI_ARRIVE_POS_ERR
           && fabsf(pMotor->v_ref) < DJI_ARRIVE_VEL_MAX)
        {
            pMotor->pos_ref   = pMotor->tgt_pos;
            pMotor->v_ref     = 0.0f;
            pMotor->plan_done = 1u;
        }

        /* ---- ③ 位置环: 轨迹跟踪误差 -> 速度修正 ---- */
        PID_Set_Target(&pMotor->position_pid, pMotor->pos_ref);
        float pos_out = PID_Calculate(&pMotor->position_pid, pMotor->RX_Frame.total_out_rad);

        /* ---- ④ 规划速度前馈直通 + 速度环 ---- */
        float vel_tgt = pos_out + DJI_PLAN_KFF * pMotor->v_ref;
        PID_Set_Target(&pMotor->speed_pid, vel_tgt);
        DJ_Set_Current(pMotor, (int16_t)PID_Calculate(&pMotor->speed_pid, pMotor->RX_Frame.omega_filt));
    }
    DJ_Send_Frame(pMotor);
}
