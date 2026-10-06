#include "power_bsp.h"

/*******************机器人地盘动力*************************/
void Wheel_config(float spd1, float spd2, float spd3, float spd4, float coeff)
{
    Motor_3519_Speed(&hcan1, spd1 * coeff, 0x205);
    osDelay(1);
    Motor_3519_Speed(&hcan1, spd2 * coeff, 0x206);
    osDelay(1);
    Motor_3519_Speed(&hcan1, spd3 * coeff, 0x207);
    osDelay(1);
    Motor_3519_Speed(&hcan1, spd4 * coeff, 0x208);
    osDelay(1);
}
/*******************机器人底盘履带动力*************************/
void Track_config(float frontPos, float frontSpd, float rearPos, float rearSpd)
{

    Motor_3519_Position(&hcan2, -frontPos, frontSpd, 0x101);
    osDelay(1);
    Motor_3519_Position(&hcan2, rearPos, rearSpd, 0x102);
    osDelay(1);
    Motor_3519_Position(&hcan2, -rearPos, rearSpd, 0x103);
    osDelay(1);
    Motor_3519_Position(&hcan2, frontPos, frontSpd, 0x104);
    osDelay(1);
}
/*******************机器人上层机构动力*************************/
// ̧抬升机构
void Gantry_lift_config(float pos, float spd)
{

    Motor_3519_Position(&hcan2, -pos, spd, 0x105);
    osDelay(1);
    Motor_3519_Position(&hcan2, pos, spd, 0x106);
    osDelay(1);
}
// 推送机构
void Gantry_push_config(float pos, float spd)
{

    Motor_3519_Position(&hcan2, pos, spd, 0x107);
    osDelay(1);
}
// 夹取机构
void Gantry_grip_config(float pos, float spd)
{

    Motor_3519_Position(&hcan2, pos, spd, 0x108);
    osDelay(1);
}

void Grip_arm_config(float pos, float spd)
{
    Motor_3519_Position(&hcan1, pos, spd, 0x101);
    osDelay(1);
}
