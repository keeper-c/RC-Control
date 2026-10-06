#include "dvc_DMMotor.h"

#include "cmsis_os2.h"

Struct_DM_Rx DM_Rx_Buffer[DM_MOTOR_NUM];

void DM_Motor_Init(Struct_Motor_Manage *Motor,CAN_HandleTypeDef *hcan, uint8_t ID)
{
    Motor->hcan = hcan;
    Motor->ID = ID;
    Motor->Speed = 0.0f;

    DM_Motor_Start(Motor);
    DM_Motor_SendSpeed(Motor, 0.0f);
}

static void DM_Motor_SendCommand(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t cmd)
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

    CAN_Send_Data(hcan, DM_MOTOR_BASE_ID + ID, Tx_Buffer, 8);
}

void DM_Motor_Start(Struct_Motor_Manage *Motor)
{
    DM_Motor_SendCommand(Motor->hcan, Motor->ID, 0xFC);
    osDelay(1);
}

void DM_Motor_Stop(Struct_Motor_Manage *Motor)
{
    DM_Motor_SendCommand(Motor->hcan, Motor->ID, 0xFD);
    osDelay(1);
}

void DM_Motor_SendSpeed(Struct_Motor_Manage *Motor, float speed)
{
    uint8_t Tx_Buffer[8] = {0};
    Float_Union fu;
    fu.f = speed;

    Tx_Buffer[0] = fu.buf[0];
    Tx_Buffer[1] = fu.buf[1];
    Tx_Buffer[2] = fu.buf[2];
    Tx_Buffer[3] = fu.buf[3];

    CAN_Send_Data(Motor->hcan, DM_MOTOR_BASE_ID + Motor->ID, Tx_Buffer, 8);
    osDelay(1);
}
