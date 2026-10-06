//
// Created by keeper on 2026/7/23.
//
#include "Types.h"

void CAN1_Rx_Callback(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    uint8_t ID    = Rx_Buffer->Data[0] & 0x0F;
    uint8_t Error = (Rx_Buffer->Data[0] >> 4) & 0x0F;

    uint16_t Pos    = (Rx_Buffer->Data[1] << 8) | Rx_Buffer->Data[2];
    uint16_t Speed  = (Rx_Buffer->Data[3] << 4) | (Rx_Buffer->Data[4] >> 4);
    uint16_t Torque = ((Rx_Buffer->Data[4] & 0x0F) << 8) | Rx_Buffer->Data[5];

    uint8_t T_MOS   = Rx_Buffer->Data[6];
    uint8_t T_Rotor = Rx_Buffer->Data[7];

    if (ID < DM_MOTOR_NUM)
    {
        DM_Rx_Buffer[ID].ID      = ID;
        DM_Rx_Buffer[ID].Error   = Error;
        DM_Rx_Buffer[ID].Pos     = Pos;
        DM_Rx_Buffer[ID].Speed   = Speed;
        DM_Rx_Buffer[ID].Torque  = Torque;
        DM_Rx_Buffer[ID].T_MOS   = T_MOS;
        DM_Rx_Buffer[ID].T_Rotor = T_Rotor;
    }
}

void Start_CAN_Task(void *argument)
{
    for(;;)
    {
        osDelay(1);
    }
}