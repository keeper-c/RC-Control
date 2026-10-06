#ifndef __CAN_BSP_H
#define __CAN_BSP_H
#include "main.h"
typedef struct
{
    int8_t temper;
    int16_t iq;
    int16_t speed;
    uint16_t encoder;
    uint16_t last_encoder_value;
    int cumulative_angle;
} M3508;

extern M3508 C1Motor_1, C1Motor_2, C1Motor_3, C1Motor_4, C1Motor_5, C1Motor_6, C1Motor_7, C1Motor_8;
extern M3508 C2Motor_1, C2Motor_2, C2Motor_3, C2Motor_4, C2Motor_5, C2Motor_6, C2Motor_7, C2Motor_8;

extern int16_t out1, out2, out3, out4, out5, out6, out7, out8;
extern int16_t put1, put2, put3, put4, put5, put6, put7, put8;

void CAN_Filter_Init(void);
void CAN_TransmitMessage_StdID(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data, uint8_t length);
void CAN_TransmitMessage_ExtID(CAN_HandleTypeDef *hcan, uint32_t ExtID, uint8_t *pData, uint8_t length);

void motor_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void up_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void motor_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void up_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

void motor5065(uint32_t ExtID, int32_t vec);
void get_absolute_angle(M3508 *motor);
void Dwheel_calc(float vx, float vy, float vw, float wheel_angles[4], float wheel_speeds[4]);
#endif
