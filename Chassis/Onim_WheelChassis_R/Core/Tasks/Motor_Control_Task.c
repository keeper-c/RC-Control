//
// Created by keeper on 2026/7/23.
//

#include "Types.h"
void Start_MoControl_Task(void *argument)
{
    Uart_RxFrame_t Uart_Control;

    DM_Motor_Init(&Omini_Chassis.Motor[0], &hcan1, 0x01);
    DM_Motor_Init(&Omini_Chassis.Motor[1], &hcan1, 0x02);
    DM_Motor_Init(&Omini_Chassis.Motor[2], &hcan1, 0x07);
    DM_Motor_Init(&Omini_Chassis.Motor[3], &hcan1, 0x04);

    Chassis_Init(&Omini_Chassis, DISTANCE, WHEEL_RADIUS, Onim_WheelChassis);

    for(;;)
    {
        osMessageQueueGet(UartRx_QueueHandle, &Uart_Control, 0, osWaitForever);

        int16_t Y = (int16_t)Uart_Control.Key_State[1] - (int16_t)Uart_Control.Key_State[2];
        int16_t X = (int16_t)Uart_Control.Key_State[0] - (int16_t)Uart_Control.Key_State[3];
        int16_t Omega = (int16_t)Uart_Control.Slider_Value[0] - (int16_t)Uart_Control.Slider_Value[1];

        DM_Motor_ChassisControl(&Omini_Chassis, X, Y, Omega);
    }
}