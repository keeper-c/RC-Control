//
// Created by keeper on 2026/7/23.
//

#ifndef ONIM_WHEELCHASSIS_CHASSIS_H
#define ONIM_WHEELCHASSIS_CHASSIS_H

#define DISTANCE        0.4375f
#define WHEEL_RADIUS    0.075f
#define SQRT2_2         0.7071f

#include "dvc_DMMotor.h"

typedef enum
{
    Onim_WheelChassis,
    Mecanum_WheelChassis,
}Enum_Type_WheelChassis;

typedef struct
{
    Struct_Motor_Manage Motor[DM_MOTOR_NUM];
    float Distance;
    float Wheel_Radius;
    Enum_Type_WheelChassis Type;

}Struct_Chassis;

extern Struct_Chassis Omini_Chassis;

void Chassis_Init(Struct_Chassis *Chassis, float Distance, float Wheel_Radius, Enum_Type_WheelChassis Type);
void DM_Motor_ChassisControl(Struct_Chassis *Chassis, float X, float Y, float Omega);

#endif //ONIM_WHEELCHASSIS_CHASSIS_H