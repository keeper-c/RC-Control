#ifndef CAN_BSP_H
#define CAN_BSP_H
#include "main.h"
#include "can.h"

#define count_F 0.8
#define count_T 0.006
#define CAN_CHASSIS_ALL_ID 0x200
#define CAN_CHASSIS_ALL_UP_ID 0x1FF
#define CAN_CHASSIS_MODE_POSITION2 0xA4
#define CAN_3508_M1_ID     0x201
#define CAN_3508_M2_ID     0x202
#define CAN_3508_M3_ID     0x203
#define CAN_3508_M4_ID     0x204
#define CAN_3508_M5_ID     0x205
#define CAN_3508_M6_ID     0x206
#define CAN_3508_M7_ID     0x207
#define CAN_3508_M8_ID     0x208
#define CAN_RS02_MODE_POSITION 1
#define CAN_RS02_POSITION_SPEED 0x7024
#define CAN_RS02_POSITION_A_SPEED 0x7025
#define CAN_RS02_POSITION 0x7016
#define CAN_RS02_BACK_EXTID (0x02<<24)|(0x80<<16)|(0x01<<8)|0xFD
#define CAN_3519_M1_ID 0x01
#define CAN_3519_M2_ID 0x02
#define CAN_3519_M3_ID 0x03
#define CAN_3519_M4_ID 0x04
#define CAN_3519_M5_ID 0x05
#define CAN_3519_M6_ID 0x06
#define CAN_3519_M7_ID 0x07
#define CAN_3519_M8_ID 0x08
#define CAN_3519_MODE_SPEED 0x03
#define CAN_3519_MODE_POSITION 0x02
typedef struct
{
    uint16_t ecd;                 //��ǰ������ֵ
    int16_t speed_rpm;           //���ת��
    int16_t given_current;       //��ǰ����ֵ
    int8_t temperate;          //�¶�
    uint16_t last_ecd;          //�ϴα�����ֵ
	int32_t total_angle;         //��ǰ������ֵ
} motor_measure_t;

typedef struct
{
    uint16_t ecd;                 //��ǰ������ֵ
    int16_t speed_rpm;           //���ת��
    int16_t given_current;       //��ǰ����ֵ
    int8_t temperate;          //�¶�
    uint16_t last_ecd;          //�ϴα�����ֵ
} motor_k_tech_measure_t;

typedef struct
{
    uint16_t angle;                 //�Ƕ� 0��65535 ��Ӧ-4�е�4��
    uint16_t speed_rpm;           //���ת�� ���ٶ� 0��65535��Ӧ-44��44rad/s
    uint16_t temperate;          //�¶�*10 
    uint16_t torque;             //���� 0��65535��Ӧ -17��17Nm
    uint16_t last_angle;          //�ϴα�����ֵ
} motor_RS02_measure_t;

typedef struct
{
    uint8_t err;             //�����룬���Բ���
    uint8_t id;               //CAN����λid
    int16_t position;          //λ��
    int16_t speed_rpm;           //�ٶ�
    uint8_t t_mos;          //������ MOS ��ƽ���¶�
    uint16_t torque;          //����
    uint8_t t_rotor;          //����ڲ���Ȧ��ƽ���¶�
} motor_3519_measure_t;

extern motor_measure_t motor_chassis[8];
extern motor_k_tech_measure_t motor_k_tech1;
extern motor_RS02_measure_t motor_rs02;
extern motor_3519_measure_t motor_3519[8];
extern motor_3519_measure_t motor_3519_Buffer;
/* void Driver_M(float vx, float vy, float vw, float speed[4]); */
void Can_Filter_Init(void);
void Can_Notify_Enable(void);
void Driver_M(float X, float Y, float Yaw, float speed[4]);
void Get_Motor_Measure(motor_measure_t *motor,const uint8_t data[]);
void Get_Motor_K_TECH_Measure(motor_k_tech_measure_t *motor,const uint8_t data[]);
void Get_Motor_3519_Measure(motor_3519_measure_t *motor,const uint8_t data[]);
void Can1_Sendmassage_classis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Can2_Sendmassage_classis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Can1_SendUpmassage_classis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Can2_SendUpmassage_classis(int16_t motor1,int16_t motor2,int16_t motor3,int16_t motor4);
void Motor_K_TECH(int32_t Position,uint16_t MaxSpeed);
void Motor_Position_Rx02_Init(uint8_t Mode,float speed,uint16_t index_S,float A_speed,uint16_t index_A);
void Motor_RS02_SetPosition(float Position,uint16_t index_P);
void Motor_3519_Start(CAN_HandleTypeDef *hcan,uint16_t CAN_Target_ID);
void Motor_3519_End(uint16_t CAN_Target_ID);
void Motor_3519_Mode(CAN_HandleTypeDef *hcan,uint32_t mode,uint16_t CAN_Target_ID);
void Motor_3519_Init(CAN_HandleTypeDef *hcan,uint32_t mode,uint16_t CAN_Target_ID);
void Motor_3519_Speed(CAN_HandleTypeDef *hcan,float Speed,uint16_t CAN_Target_ID);
void Motor_3519_Position(CAN_HandleTypeDef *hcan,float Position,float Speed,uint16_t CAN_Target_ID);
void UpdateEncoder(motor_measure_t *motor_Classis);
/* void Conversion(int16_t Speed[4],float Vx, float Vy, float Vw); */
#endif
