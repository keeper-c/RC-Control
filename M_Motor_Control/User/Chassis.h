#ifndef __CHASSIS_H__
#define __CHASSIS_H__   

#include "main.h"
#include "drv_DJ_Control.h"
#include "drv_DM_Control.h"


#define RC_MAX  100.0f   /* 遥控器摇杆满量程(DJI遥控器右左摇±100) */
#define MORE_MOTOR_NUM 5

typedef enum
{
   Chassis_Mode_Omni,
   Chassis_Mode_Mecanum,
}Chassis_Mode_t;

typedef enum
{
    Chassis_Motor_DJ,
    Chassis_Motor_DM,
}Chassis_Motor_t;

typedef struct
{
    DJ_Motor_t DJMotor[4];
    DM_Motor_t DM_Motor[4];
    Chassis_Mode_t Mode;
    Chassis_Motor_t Motor;
    float Max_Speed;   /* 最大轮速(rad/s), 遥控器满量程对应的目标速度 */
    int16_t x;
    int16_t y;
    int16_t z;
}Chassis_t;

typedef struct
{
    DJ_Motor_t DJMotor[MORE_MOTOR_NUM];
    DM_Motor_t DM_Motor[MORE_MOTOR_NUM];
    Chassis_Motor_t Motor;
    float Target[MORE_MOTOR_NUM];
}More_Motor_t;

void Chassis_Init(Chassis_t *Chassis, Chassis_Motor_t motor, Chassis_Mode_t mode, CAN_HandleTypeDef* hcan, uint16_t* ID);
void Chassis_Control(Chassis_t *Chassis);//内含HAL_Delay，并且中断优先级未配置，建议在主循环中调用或使用freertos
void Chassis_Set_Target(Chassis_t *Chassis, int16_t x, int16_t y, int16_t z);

void More_Motor_Control(More_Motor_t *More_Motor);
void More_Motor_Set_Target(More_Motor_t *More_Motor, float *target);
void More_Motor_Init(More_Motor_t *More_Motor, Chassis_Motor_t mode, CAN_HandleTypeDef* hcan, uint16_t* ID);

#endif
