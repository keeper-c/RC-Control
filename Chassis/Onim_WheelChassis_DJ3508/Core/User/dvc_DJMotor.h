#ifndef DVC_DJMOTOR_H
#define DVC_DJMOTOR_H

#include "drv_can.h"
#include "pid.h"

#define DJ_MOTOR_NUM        4
#define DJ_MOTOR_GEAR_RATIO 19.0f
#define PI                  3.14159265358979f

typedef struct
{
    float angle_rad;
    float speed_radps;
    int16_t torque;
    uint8_t temperature;
} Struct_DJ_Rx;

typedef enum
{
    CONTROL_MODE_OMEGA,
    CONTROL_MODE_ANGLE
} Enum_ControlMode;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint16_t ID;
    uint8_t Data[8];
    Struct_PID pid_speed[DJ_MOTOR_NUM];
    Struct_PID pid_angle[DJ_MOTOR_NUM];
    Struct_DJ_Rx Rx_Buffer[DJ_MOTOR_NUM];
    float total_angle[DJ_MOTOR_NUM];
    uint16_t last_angle[DJ_MOTOR_NUM];
} Struct_Motor_Manage;

void DJ_Motor_Init(Struct_Motor_Manage *Motor, CAN_HandleTypeDef *hcan, uint16_t ID);
void DJ_Motor_Send(Struct_Motor_Manage *Motor, int16_t Torque1, int16_t Torque2, int16_t Torque3, int16_t Torque4);
void DJ_Motor_Receive(Struct_DJ_Rx *Rx_Buffer, const uint8_t *Data);
void DJ_Motor_Control(Struct_Motor_Manage *Motor, const float *Target, Enum_ControlMode Mode);

#endif
