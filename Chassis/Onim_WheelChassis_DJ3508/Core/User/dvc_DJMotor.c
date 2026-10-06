#include "dvc_DJMotor.h"

#define ENCODER_RESOLUTION 8192.0f
#define RAD_PER_ENCODER    (2.0f * PI / ENCODER_RESOLUTION)
#define SPEED_TO_RADPS     (2.0f * PI / 60.0f)

void DJ_Motor_Init(Struct_Motor_Manage *Motor, CAN_HandleTypeDef *hcan, uint16_t ID)
{
    Motor->hcan = hcan;
    Motor->ID = ID;
}

void DJ_Motor_Send(Struct_Motor_Manage *Motor, int16_t Torque1, int16_t Torque2, int16_t Torque3, int16_t Torque4)
{
    Motor->Data[0] = (uint8_t)(Torque1 >> 8);
    Motor->Data[1] = (uint8_t)(Torque1 & 0xFF);
    Motor->Data[2] = (uint8_t)(Torque2 >> 8);
    Motor->Data[3] = (uint8_t)(Torque2 & 0xFF);
    Motor->Data[4] = (uint8_t)(Torque3 >> 8);
    Motor->Data[5] = (uint8_t)(Torque3 & 0xFF);
    Motor->Data[6] = (uint8_t)(Torque4 >> 8);
    Motor->Data[7] = (uint8_t)(Torque4 & 0xFF);

    CAN_Send_Data(Motor->hcan, Motor->ID, Motor->Data, 8);
}

void DJ_Motor_Receive(Struct_DJ_Rx *Rx_Buffer, const uint8_t *Data)
{
    uint16_t angle_raw = (uint16_t)(Data[0] << 8 | Data[1]);
    int16_t speed_raw = (int16_t)(Data[2] << 8 | Data[3]);

    Rx_Buffer->angle_rad = (float)angle_raw * RAD_PER_ENCODER / DJ_MOTOR_GEAR_RATIO;
    Rx_Buffer->speed_radps = (float)speed_raw * SPEED_TO_RADPS / DJ_MOTOR_GEAR_RATIO;
    Rx_Buffer->torque = (int16_t)(Data[4] << 8 | Data[5]);
    Rx_Buffer->temperature = Data[6];
}

void DJ_Motor_Control(Struct_Motor_Manage *Motor, const float *Target, Enum_ControlMode Mode)
{
    if (Mode == CONTROL_MODE_ANGLE)
    {
        for (int i = 0; i < DJ_MOTOR_NUM; i++)
        {
            uint16_t current_raw = (uint16_t)(Motor->Rx_Buffer[i].angle_rad * DJ_MOTOR_GEAR_RATIO / RAD_PER_ENCODER + 0.5f);
            int16_t diff = (int16_t)(current_raw - Motor->last_angle[i]);

            if (diff > 4096)
            {
                Motor->total_angle[i] -= 2.0f * PI / DJ_MOTOR_GEAR_RATIO;
            }
            else if (diff < -4096)
            {
                Motor->total_angle[i] += 2.0f * PI / DJ_MOTOR_GEAR_RATIO;
            }

            Motor->total_angle[i] += (float)diff * RAD_PER_ENCODER / DJ_MOTOR_GEAR_RATIO;
            Motor->last_angle[i] = current_raw;

            float angle_error = Target[i] - Motor->total_angle[i];
            PID_SetTarget(&Motor->pid_angle[i], angle_error);
            PID_SetCurrent(&Motor->pid_angle[i], 0.0f);
            float target_speed = PID_Calculate(&Motor->pid_angle[i]);

            PID_SetTarget(&Motor->pid_speed[i], target_speed);
            PID_SetCurrent(&Motor->pid_speed[i], Motor->Rx_Buffer[i].speed_radps);
            PID_Calculate(&Motor->pid_speed[i]);
        }
    }
    else if (Mode == CONTROL_MODE_OMEGA)
    {
        for (int i = 0; i < DJ_MOTOR_NUM; i++)
        {
            PID_SetTarget(&Motor->pid_speed[i], Target[i]);
            PID_SetCurrent(&Motor->pid_speed[i], Motor->Rx_Buffer[i].speed_radps);
            PID_Calculate(&Motor->pid_speed[i]);
        }
    }

    for (int i = 0; i < DJ_MOTOR_NUM; i++)
    {
        if (Motor->pid_speed[i].Output > 16384.0f) Motor->pid_speed[i].Output = 16384.0f;
        if (Motor->pid_speed[i].Output < -16384.0f) Motor->pid_speed[i].Output = -16384.0f;
    }

    DJ_Motor_Send(Motor,
                  (int16_t)Motor->pid_speed[0].Output,
                  (int16_t)Motor->pid_speed[1].Output,
                  (int16_t)Motor->pid_speed[2].Output,
                  (int16_t)Motor->pid_speed[3].Output);
}
