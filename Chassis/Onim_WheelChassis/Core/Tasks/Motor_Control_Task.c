//
// Created by keeper on 2026/7/23.
//

#include "Types.h"
void Start_MoControl_Task(void *argument)
{
    Uart_RxFrame_t Uart_Control = {0};

    DM_Motor_Init(&Omini_Chassis.Motor[0], &hcan1, 0x01);
    DM_Motor_Init(&Omini_Chassis.Motor[1], &hcan1, 0x02);
    DM_Motor_Init(&Omini_Chassis.Motor[2], &hcan1, 0x03);
    DM_Motor_Init(&Omini_Chassis.Motor[3], &hcan1, 0x04);

    Chassis_Init(&Omini_Chassis, DISTANCE, WHEEL_RADIUS, Onim_WheelChassis);

    for(;;)
    {
        osMessageQueueGet(UartRx_QueueHandle, &Uart_Control, 0, osWaitForever);
        DM_Motor_ChassisControl(&Omini_Chassis, Uart_Control.X, Uart_Control.Y, Uart_Control.Omega);
    }
}