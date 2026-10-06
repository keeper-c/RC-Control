//
// Created by keeper on 2026/7/23.
//

#include "Chassis.h"

Struct_Chassis Omini_Chassis = {0};

void Chassis_Init(Struct_Chassis *Chassis, float Distance, float Wheel_Radius, Enum_Type_WheelChassis Type)
{
    Chassis->Distance = Distance;
    Chassis->Wheel_Radius = Wheel_Radius;
    Chassis->Type = Type;
}

void DM_Motor_ChassisControl(Struct_Chassis *Chassis, float X, float Y, float Omega)
{
    float Omega_coef = Chassis->Distance / Chassis->Wheel_Radius;

    float Motor1_Speed = 0;
    float Motor2_Speed = 0;
    float Motor3_Speed = 0;
    float Motor4_Speed = 0;

    if (Chassis->Type == Onim_WheelChassis)
    {
        Motor1_Speed = SQRT2_2 * (-X + Y + Omega * Omega_coef);
        Motor2_Speed = SQRT2_2 * (-X - Y + Omega * Omega_coef);
        Motor3_Speed = SQRT2_2 * ( X - Y + Omega * Omega_coef);
        Motor4_Speed = SQRT2_2 * ( X + Y + Omega * Omega_coef);
    }
    else if (Chassis->Type == Mecanum_WheelChassis)
    {
        Motor1_Speed =  X - Y - Omega * Omega_coef;  // 前左 (Front-Left)
        Motor2_Speed =  X + Y + Omega * Omega_coef;  // 前右 (Front-Right)
        Motor3_Speed =  X + Y - Omega * Omega_coef;  // 后左 (Rear-Left)
        Motor4_Speed =  X - Y + Omega * Omega_coef;  // 后右 (Rear-Right)
    }

    DM_Motor_SendSpeed(&Chassis->Motor[0], Motor1_Speed);
    DM_Motor_SendSpeed(&Chassis->Motor[1], Motor2_Speed);
    DM_Motor_SendSpeed(&Chassis->Motor[2], Motor3_Speed);
    DM_Motor_SendSpeed(&Chassis->Motor[3], Motor4_Speed);
}