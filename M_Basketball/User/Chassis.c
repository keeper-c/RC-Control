#include "Chassis.h"
#include <math.h>



void Chassis_Init(Chassis_t *Chassis, Chassis_Motor_t motor, Chassis_Mode_t mode, CAN_HandleTypeDef* hcan, uint16_t* ID)
{

    Chassis->x = 0;
    Chassis->y = 0;
    Chassis->z = 0;

    Chassis->Mode = mode;
    Chassis->Motor = motor;
    Chassis->Max_Speed = 30.0f;  /* 默认最大轮速 30 rad/s, 按电机/负载调整 */

    if(motor == Chassis_Motor_DJ)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            DJ_Motor_Init(&Chassis->DJMotor[i], hcan, ID[i], DJ_MOTOR_MODE_SPEED, MOTOR_REDUCTION_RATIO, DJ_Send_Data_0x200);
        }
    }
    else if(motor == Chassis_Motor_DM)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            DM_Motor_Init(&Chassis->DM_Motor[i], hcan, ID[i], DM_Mode_Speed, DM_MIT_P_MAX, DM_MIT_V_MAX, DM_MIT_T_MAX, DM_MIT_KP_MAX, DM_MIT_KD_MAX);
        }
    }
}

void More_Motor_Init(More_Motor_t *More_Motor, Chassis_Motor_t motor, CAN_HandleTypeDef* hcan, uint16_t* ID)
{
    More_Motor->Motor = motor;
    for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
    {
        More_Motor->Target[i] = 0.0f;
    }
    if(motor == Chassis_Motor_DM)
    {
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            DM_Motor_Init(&More_Motor->DM_Motor[i], hcan, ID[i], DM_Mode_PS, DM_MIT_P_MAX, DM_MIT_V_MAX, DM_MIT_T_MAX, DM_MIT_KP_MAX, DM_MIT_KD_MAX);
        }
    }
    else if(motor == Chassis_Motor_DJ)
    {
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            DJ_Motor_Init(&More_Motor->DJMotor[i], hcan, ID[i], DJ_MOTOR_MODE_POSITION, MOTOR_REDUCTION_RATIO, DJ_Send_Data_0x200);
        }
    }

}

void More_Motor_Set_Target(More_Motor_t *More_Motor, float *target)
{
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            More_Motor->Target[i] = target[i];
        }
}

void Chassis_Set_Target(Chassis_t *Chassis, int16_t x, int16_t y, int16_t z)
{
    Chassis->x = x;
    Chassis->y = y;
    Chassis->z = z;
}

void More_Motor_Control(More_Motor_t *More_Motor)
{
    if(More_Motor->Motor == Chassis_Motor_DM)
    {
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            DM_Set_PS(&More_Motor->DM_Motor[i], More_Motor->Target[i], 3.0f);
            DM_Send_Frame(&More_Motor->DM_Motor[i]);
        }
    }
    else if(More_Motor->Motor == Chassis_Motor_DJ)
    {
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            DJ_Set_Target(&More_Motor->DJMotor[i], More_Motor->Target[i]);
        }
        for (uint8_t i = 0; i < MORE_MOTOR_NUM; i++)
        {
            DJ_Motor_Control(&More_Motor->DJMotor[i]);
        }
    }
}

void Chassis_Control(Chassis_t *Chassis)
{
    float vx = (float)Chassis->x / RC_MAX;
    float vy = (float)Chassis->y / RC_MAX;
    float vw = (float)Chassis->z / RC_MAX;

    if(vx >  1.0f) vx =  1.0f;  if(vx < -1.0f) vx = -1.0f;
    if(vy >  1.0f) vy =  1.0f;  if(vy < -1.0f) vy = -1.0f;
    if(vw >  1.0f) vw =  1.0f;  if(vw < -1.0f) vw = -1.0f;

    float wheel[4];

    if(Chassis->Mode == Chassis_Mode_Mecanum)
    {
		wheel[0] = -vx - vy - vw; // FL 左前
		wheel[1] = -vx + vy - vw; // FR 右前
		wheel[2] = +vx + vy - vw; // RR 右后
		wheel[3] = +vx - vy - vw; // RL 左后
    }
    else if(Chassis->Mode == Chassis_Mode_Omni)
    {
        /* 全向轮 +型逆运动学:
           轮序: 0=前  1=右  2=后  3=左
           纯前进(vx=1): 前正转, 后反转
           纯右移(vy=1): 右正转, 左反转
           纯左转(vw=1): 四轮正转 */
        wheel[0] =  vx + vw;       /* 前 */
        wheel[1] =  vy + vw;       /* 右 (修正: 原 -vy 反了) */
        wheel[2] = -vx + vw;       /* 后 */
        wheel[3] = -vy + vw;       /* 左 (修正: 原 vy 反了) */
    }

    float max_abs = 0.0f;
    for(uint8_t i = 0; i < 4; i++)
    {
        if(fabsf(wheel[i]) > max_abs) max_abs = fabsf(wheel[i]);
    }
    if(max_abs > 1.0f)
    {
        float scale = 1.0f / max_abs;
        for(uint8_t i = 0; i < 4; i++) wheel[i] *= scale;
    }

    if(Chassis->Motor == Chassis_Motor_DJ)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            DJ_Set_Target(&Chassis->DJMotor[i], wheel[i] * Chassis->Max_Speed);
        } 
        for(uint8_t i = 0; i < 4; i++)
        {
            DJ_Motor_Control(&Chassis->DJMotor[i]);
        }
    }
    else if(Chassis->Motor == Chassis_Motor_DM)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            DM_Set_S(&Chassis->DM_Motor[i], wheel[i] * Chassis->Max_Speed);
            DM_Send_Frame(&Chassis->DM_Motor[i]);
        }
    }
}

