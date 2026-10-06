#ifndef CHASSIS_H
#define CHASSIS_H

#include "dvc_DJMotor.h"

#define CHASSIS_DISTANCE     0.325f
#define CHASSIS_WHEEL_RADIUS 0.075f
#define CHASSIS_CONTROL_DT   0.001f

#define SQRT2_2              0.7071f

typedef struct
{
    Struct_Motor_Manage Motor;
    float Distance;
    float Wheel_Radius;
} Struct_Chassis;

extern Struct_Chassis Omini_Chassis;

void Chassis_Init(Struct_Chassis *Chassis, CAN_HandleTypeDef *hcan, uint16_t Motor_ID, float Distance, float Wheel_Radius);
void Chassis_Control(Struct_Chassis *Chassis, float X, float Y, float Omega);

#endif
