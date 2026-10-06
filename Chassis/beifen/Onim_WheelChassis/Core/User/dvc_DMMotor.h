#ifndef DVC_DMMOTOR_H
#define DVC_DMMOTOR_H

#include "drv_can.h"

#define DM_MOTOR_BASE_ID    0x200
#define DM_MOTOR_NUM        4

typedef union
{
    float f;
    uint8_t buf[4];
} Float_Union;


typedef struct
{
    uint8_t ID;
    uint8_t Error;
    uint16_t Pos;
    uint16_t Speed;
    uint16_t Torque;
    uint8_t T_MOS;
    uint8_t T_Rotor;

} Struct_DM_Rx;

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint8_t ID;
    float Speed;

}Struct_Motor_Manage;

extern Struct_DM_Rx DM_Rx_Buffer[DM_MOTOR_NUM];

void DM_Motor_Init(Struct_Motor_Manage *Motor,CAN_HandleTypeDef *hcan, uint8_t ID);
void DM_Motor_Start(Struct_Motor_Manage *Motor);
void DM_Motor_Stop(Struct_Motor_Manage *Motor);
void DM_Motor_SendSpeed(Struct_Motor_Manage *Motor, float speed);

#endif