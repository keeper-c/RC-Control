#include "Chassis.h"

Struct_Chassis Omini_Chassis = {0};

void Chassis_Init(Struct_Chassis *Chassis, CAN_HandleTypeDef *hcan, uint16_t Motor_ID, float Distance, float Wheel_Radius)
{
    Chassis->Distance = Distance;
    Chassis->Wheel_Radius = Wheel_Radius;

    DJ_Motor_Init(&Chassis->Motor, hcan, Motor_ID);

    for (int i = 0; i < DJ_MOTOR_NUM; i++)
    {
        PID_Init(&Chassis->Motor.pid_speed[i], 200.0f, 20.0f, 0.0f, 150.0f, 500.0f, 8192.0f, CHASSIS_CONTROL_DT);
        PID_Init(&Chassis->Motor.pid_angle[i], 5.0f, 0.5f, 0.0f, 150.0f, 500.0f, 2000.0f, CHASSIS_CONTROL_DT);
    }
}

void Chassis_Control(Struct_Chassis *Chassis, float X, float Y, float Omega)
{
    float L = Chassis->Distance;
    float r = Chassis->Wheel_Radius;

    float v1 = (-1.0f * SQRT2_2 / 2.0f * X + 1.0f * SQRT2_2 / 2.0f * Y + Omega * L) / r;
    float v2 = (-1.0f * SQRT2_2 / 2.0f * X - 1.0f * SQRT2_2 / 2.0f * Y + Omega * L) / r;
    float v3 = (1.0f * SQRT2_2 / 2.0f * X - 1.0f * SQRT2_2 / 2.0f * Y + Omega * L) / r;
    float v4 = (1.0f * SQRT2_2 / 2.0f * X + 1.0f * SQRT2_2 / 2.0f * Y + Omega * L) / r;

    float wheel_speed[4] = {v1, v2, v3, v4};

    DJ_Motor_Control(&Chassis->Motor, wheel_speed, CONTROL_MODE_OMEGA);
}
