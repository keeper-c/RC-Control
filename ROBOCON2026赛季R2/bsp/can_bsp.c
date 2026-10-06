#include "can_bsp.h"

// #define ENCODER_NUM_ROUND     8192     //编码器总共刻度值
motor_measure_t motor_chassis[8];
motor_k_tech_measure_t motor_k_tech1;
motor_RS02_measure_t motor_rs02;
motor_3519_measure_t motor_3519[8];
motor_3519_measure_t motor_3519_Buffer;
static CAN_TxHeaderTypeDef Tx1_message;
static CAN_TxHeaderTypeDef Tx1_upmessage;
static CAN_TxHeaderTypeDef Tx2_message;
static CAN_TxHeaderTypeDef Tx2_upmessage;
static CAN_TxHeaderTypeDef Tx_K_TECH_message;
static CAN_TxHeaderTypeDef Tx_RS02_message;
static CAN_TxHeaderTypeDef Tx_DM3519_message;
static uint8_t Can1_send_data[8];
static uint8_t Can1_send_up_data[8];
static uint8_t Can2_send_data[8];
static uint8_t Can2_send_up_data[8];
static uint8_t K_TECH_send_data[8];
static uint8_t RS02_send_data[8];
static uint8_t DM3519_send_data[8];
void Can_Filter_Init(void)
{
    CAN_FilterTypeDef canFilterConfig1;
    CAN_FilterTypeDef canFilterConfig2;

    canFilterConfig1.FilterBank = 0;
    canFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig1.FilterIdHigh = 0x0000;
    canFilterConfig1.FilterIdLow = 0x0000;
    canFilterConfig1.FilterMaskIdHigh = 0x0000;
    canFilterConfig1.FilterMaskIdLow = 0x0000;
    canFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterConfig1.FilterActivation = ENABLE;

    canFilterConfig2.FilterBank = 14;
    canFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig2.FilterIdHigh = 0x0000;
    canFilterConfig2.FilterIdLow = 0x0000;
    canFilterConfig2.FilterMaskIdHigh = 0x0000;
    canFilterConfig2.FilterMaskIdLow = 0x0000;
    canFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterConfig2.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig1);
    HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig2);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}

void Can_Notify_Enable(void)
{
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// masterid是0x00
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
void Get_Motor_3519_Measure(motor_3519_measure_t *motor, const uint8_t data[])
{
    motor->id = data[0] & 0x0F;
    motor->err = data[0] >> 4;
    motor->position = (data[1] << 8) | data[2];
    motor->speed_rpm = ((data[3] << 4) | data[4] >> 4);
    motor->torque = (data[4] & 0x0F) << 8 | data[5];
    motor->t_mos = data[6];
    motor->t_rotor = data[7];
    switch (motor->id)
    {
    case 0x01:
        motor_3519[0] = *motor;
        break;
    case 0x02:
        motor_3519[1] = *motor;
        break;
    case 0x03:
        motor_3519[2] = *motor;
        break;
    case 0x04:
        motor_3519[3] = *motor;
        break;
    }
}

// 灵足电机数据解算RS02
void Get_Motor_RS02_Measure(motor_RS02_measure_t *motor, const uint8_t data[])
{
    motor->last_angle = motor->angle;
    motor->angle = (data[0] << 8) | data[1];
    motor->speed_rpm = (data[2] << 8) | data[3];
    motor->torque = (data[4] << 8) | data[5];
    motor->temperate = (data[6] << 8) | data[7];
}

// 瓴控电机数据解析 减速比36:1 输入1296000为轴转一圈
void Get_Motor_K_TECH_Measure(motor_k_tech_measure_t *motor, const uint8_t data[])
{
    if (data[0] == CAN_CHASSIS_MODE_POSITION2)
    {
        motor->last_ecd = motor->ecd;
        motor->temperate = data[1];
        motor->given_current = (data[3] << 8) | data[2];
        motor->speed_rpm = (data[5] << 8) | data[4];
        motor->ecd = (uint16_t)((data[7] << 8) | data[6]);
    }
    else
    {
        return;
    }
}
// 3508和3519一拖四解析
void Get_Motor_Measure(motor_measure_t *motor, const uint8_t data[])
{
    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)((data[0] << 8) | data[1]);
    motor->speed_rpm = ((data[2] << 8) | data[3]);
    motor->given_current = (data[4] << 8) | data[5];
    motor->temperate = data[6];
}
// 防止跳圈
void UpdateEncoder(motor_measure_t *motor_Classis)
{
    uint16_t encoder;
    int16_t delta_encoder;
    encoder = motor_Classis->ecd;
    delta_encoder = encoder - motor_Classis->last_ecd;

    if (delta_encoder > 4096)
    {
        // 正转一圈
        delta_encoder -= 8192;
    }
    if (delta_encoder < -4096)
    {
        // 反转一圈
        delta_encoder += 8192;
    }
    motor_Classis->last_ecd = encoder;
    motor_Classis->total_angle += delta_encoder;
}
// 一到四
void Can1_Sendmassage_classis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Tx1_message.StdId = CAN_CHASSIS_ALL_ID;
    Tx1_message.IDE = CAN_ID_STD;
    Tx1_message.RTR = CAN_RTR_DATA;
    Tx1_message.DLC = 0x08;
    Can1_send_data[0] = motor1 >> 8;
    Can1_send_data[1] = motor1;
    Can1_send_data[2] = motor2 >> 8;
    Can1_send_data[3] = motor2;
    Can1_send_data[4] = motor3 >> 8;
    Can1_send_data[5] = motor3;
    Can1_send_data[6] = motor4 >> 8;
    Can1_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &Tx1_message, Can1_send_data, &send_mail_box);
}
// 五到八
void Can1_SendUpmassage_classis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Tx1_upmessage.StdId = CAN_CHASSIS_ALL_UP_ID;
    Tx1_upmessage.IDE = CAN_ID_STD;
    Tx1_upmessage.RTR = CAN_RTR_DATA;
    Tx1_upmessage.DLC = 0x08;
    Can1_send_up_data[0] = motor1 >> 8;
    Can1_send_up_data[1] = motor1;
    Can1_send_up_data[2] = motor2 >> 8;
    Can1_send_up_data[3] = motor2;
    Can1_send_up_data[4] = motor3 >> 8;
    Can1_send_up_data[5] = motor3;
    Can1_send_up_data[6] = motor4 >> 8;
    Can1_send_up_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &Tx1_upmessage, Can1_send_up_data, &send_mail_box);
}
// 一到四
void Can2_Sendmassage_classis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Tx2_message.StdId = CAN_CHASSIS_ALL_ID;
    Tx2_message.IDE = CAN_ID_STD;
    Tx2_message.RTR = CAN_RTR_DATA;
    Tx2_message.DLC = 0x08;
    Can2_send_data[0] = motor1 >> 8;
    Can2_send_data[1] = motor1;
    Can2_send_data[2] = motor2 >> 8;
    Can2_send_data[3] = motor2;
    Can2_send_data[4] = motor3 >> 8;
    Can2_send_data[5] = motor3;
    Can2_send_data[6] = motor4 >> 8;
    Can2_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &Tx2_message, Can2_send_data, &send_mail_box);
}
// 五到八
void Can2_SendUpmassage_classis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    Tx2_upmessage.StdId = CAN_CHASSIS_ALL_UP_ID;
    Tx2_upmessage.IDE = CAN_ID_STD;
    Tx2_upmessage.RTR = CAN_RTR_DATA;
    Tx2_upmessage.DLC = 0x08;
    Can2_send_up_data[0] = motor1 >> 8;
    Can2_send_up_data[1] = motor1;
    Can2_send_up_data[2] = motor2 >> 8;
    Can2_send_up_data[3] = motor2;
    Can2_send_up_data[4] = motor3 >> 8;
    Can2_send_up_data[5] = motor3;
    Can2_send_up_data[6] = motor4 >> 8;
    Can2_send_up_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &Tx2_upmessage, Can2_send_up_data, &send_mail_box);
}
// 瓴控关节电机 位置式控制
// 角度*3600 为对应关系
void Motor_K_TECH(int32_t Position, uint16_t MaxSpeed)
{
    uint32_t send_mail_box;
    Tx_K_TECH_message.StdId = 0x141;
    Tx_K_TECH_message.IDE = CAN_ID_STD;
    Tx_K_TECH_message.RTR = CAN_RTR_DATA;
    Tx_K_TECH_message.DLC = 0x08;
    K_TECH_send_data[0] = 0xA8;
    K_TECH_send_data[1] = 0x00;
    K_TECH_send_data[2] = MaxSpeed;
    K_TECH_send_data[3] = MaxSpeed >> 8;
    K_TECH_send_data[4] = Position;
    K_TECH_send_data[5] = Position >> 8;
    K_TECH_send_data[6] = Position >> 16;
    K_TECH_send_data[7] = Position >> 24;
    HAL_CAN_AddTxMessage(&hcan1, &Tx_K_TECH_message, K_TECH_send_data, &send_mail_box);
}

// RS02电机位置模式需要使用通信类型18 逐个参数发送
// 发送顺序 模式 使能 速度 加速度 位置
// RS02电机发送的是弧度制
// 使能RS02电机
void Motor_RS02_Start(void)
{
    uint32_t send_mail_box;
    uint32_t Extid = 0;
    Extid = (0x03) << 24 | (0xFD) << 8 | 0x01;
    Tx_RS02_message.ExtId = Extid;
    Tx_RS02_message.IDE = CAN_ID_EXT;
    Tx_RS02_message.RTR = CAN_RTR_DATA;
    Tx_RS02_message.DLC = 0x08;
    RS02_send_data[0] = 0x00;
    RS02_send_data[1] = 0x00;
    RS02_send_data[2] = 0x00;
    RS02_send_data[3] = 0x00;
    RS02_send_data[4] = 0x00;
    RS02_send_data[5] = 0x00;
    RS02_send_data[6] = 0x00;
    RS02_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&hcan1, &Tx_RS02_message, RS02_send_data, &send_mail_box);
}
// 设置位置模式
void Motor_RS02_Mode(uint8_t Mode)
{
    uint32_t send_mail_box;
    uint32_t Extid = 0;
    Extid = (0x12) << 24 | (0xFD) << 8 | 0x01;
    Tx_RS02_message.ExtId = Extid;
    Tx_RS02_message.IDE = CAN_ID_EXT;
    Tx_RS02_message.RTR = CAN_RTR_DATA;
    Tx_RS02_message.DLC = 0x08;
    RS02_send_data[0] = 0x05;
    RS02_send_data[1] = 0x70;
    RS02_send_data[2] = 0x00;
    RS02_send_data[3] = 0x00;
    RS02_send_data[4] = Mode;
    RS02_send_data[5] = 0x00;
    RS02_send_data[6] = 0x00;
    RS02_send_data[7] = 0x00;
    HAL_CAN_AddTxMessage(&hcan1, &Tx_RS02_message, RS02_send_data, &send_mail_box);
}
// 传递参数数据
void Motor_RS02_Data(float Data, uint16_t Index)
{
    uint32_t send_mail_box;
    uint32_t Extid = 0;
    Extid = (0x12) << 24 | (0xFD) << 8 | 0x01;
    Tx_RS02_message.ExtId = Extid;
    Tx_RS02_message.IDE = CAN_ID_EXT;
    Tx_RS02_message.RTR = CAN_RTR_DATA;
    Tx_RS02_message.DLC = 0x08;
    RS02_send_data[0] = Index;
    RS02_send_data[1] = Index >> 8;
    RS02_send_data[2] = 0x00;
    RS02_send_data[3] = 0x00;
    memcpy(&RS02_send_data[4], &Data, 4);
    HAL_CAN_AddTxMessage(&hcan1, &Tx_RS02_message, RS02_send_data, &send_mail_box);
}
// 已经封装的初始化函数
void Motor_Position_Rx02_Init(uint8_t Mode, float speed, uint16_t index_S, float A_speed, uint16_t index_A)
{
    Motor_RS02_Mode(Mode);
    Motor_RS02_Start();
    Motor_RS02_Data(speed, index_S);
    Motor_RS02_Data(A_speed, index_A);
}
// 单独调节位置
void Motor_RS02_SetPosition(float Position, uint16_t index_P)
{
    Motor_RS02_Data(Position, index_P);
}

// CAN_ID为电机ID不需要加任何偏转
void Motor_3519_Start(CAN_HandleTypeDef *hcan, uint16_t CAN_Target_ID)
{
    uint32_t send_mail_box;
    Tx_DM3519_message.StdId = CAN_Target_ID;
    Tx_DM3519_message.IDE = CAN_ID_STD;
    Tx_DM3519_message.RTR = CAN_RTR_DATA;
    Tx_DM3519_message.DLC = 0x08;
    DM3519_send_data[0] = 0xFF;
    DM3519_send_data[1] = 0xFF;
    DM3519_send_data[2] = 0xFF;
    DM3519_send_data[3] = 0xFF;
    DM3519_send_data[4] = 0xFF;
    DM3519_send_data[5] = 0xFF;
    DM3519_send_data[6] = 0xFF;
    DM3519_send_data[7] = 0xFC;
    HAL_CAN_AddTxMessage(hcan, &Tx_DM3519_message, DM3519_send_data, &send_mail_box);
}

void Motor_3519_End(uint16_t CAN_Target_ID)
{
    uint32_t send_mail_box;
    Tx_DM3519_message.StdId = CAN_Target_ID;
    Tx_DM3519_message.IDE = CAN_ID_STD;
    Tx_DM3519_message.RTR = CAN_RTR_DATA;
    Tx_DM3519_message.DLC = 0x08;
    DM3519_send_data[0] = 0xFF;
    DM3519_send_data[1] = 0xFF;
    DM3519_send_data[2] = 0xFF;
    DM3519_send_data[3] = 0xFF;
    DM3519_send_data[4] = 0xFF;
    DM3519_send_data[5] = 0xFF;
    DM3519_send_data[6] = 0xFF;
    DM3519_send_data[7] = 0xFD;
    HAL_CAN_AddTxMessage(&hcan1, &Tx_DM3519_message, DM3519_send_data, &send_mail_box);
}

void Motor_3519_Mode(CAN_HandleTypeDef *hcan, uint32_t mode, uint16_t CAN_Target_ID)
{
    uint32_t send_mail_box;
    Tx_DM3519_message.StdId = 0x7FF;
    Tx_DM3519_message.IDE = CAN_ID_STD;
    Tx_DM3519_message.RTR = CAN_RTR_DATA;
    Tx_DM3519_message.DLC = 0x08;
    DM3519_send_data[0] = CAN_Target_ID;
    DM3519_send_data[1] = CAN_Target_ID >> 8;
    DM3519_send_data[2] = 0x55;
    DM3519_send_data[3] = 0x0A;
    DM3519_send_data[4] = mode;
    DM3519_send_data[5] = mode >> 8;
    DM3519_send_data[6] = mode >> 16;
    DM3519_send_data[7] = mode >> 24;
    HAL_CAN_AddTxMessage(hcan, &Tx_DM3519_message, DM3519_send_data, &send_mail_box);
}

//// 在main函数初始化完成后延时1ms后Init函数
//void Motor_3519_Init(CAN_HandleTypeDef *hcan, uint32_t mode, uint16_t CAN_Target_ID)
//{
//    Motor_3519_Mode(hcan, mode, CAN_Target_ID);
////    // 等待 TX 邮箱释放，加超时防止死等（无 ACK 时 AutoRetransmission 会一直占用邮箱）
////    uint32_t tout = 500; // 500 * 2ms = 1s 超时
////    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) < 3 && tout > 0) {
//        HAL_Delay(2);
////        tout--;
////    }
//    Motor_3519_Start(hcan, CAN_Target_ID);
//}
//在main函数初始化完成后延时1ms后Init函数
void Motor_3519_Init(CAN_HandleTypeDef *hcan,uint32_t mode,uint16_t CAN_Target_ID)
{
  Motor_3519_Mode(hcan,mode,CAN_Target_ID);
  HAL_Delay(5);
  Motor_3519_Start(hcan,CAN_Target_ID);
  HAL_Delay(5);
  Motor_3519_Start(hcan,CAN_Target_ID); // 上电首帧可能丢，重发一次
}
/* CAN 发送带重试 — mailbox 满则等 1ms 重发，防止丢包 */
static void CAN_Tx_Retry(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
    uint32_t mailbox;
    uint32_t timeout = 10; // 100ms 超时
	  
	  //掩码
	  if((pHeader->StdId & 0xF00)== 0x100 &&(pHeader->StdId & 0xF0F)>= 0x105)
    {
      while (HAL_CAN_AddTxMessage(hcan, pHeader, pData, &mailbox) != HAL_OK && timeout > 0)
      {
        HAL_Delay(1);
        timeout--;
      }
	 }
   else
  {
	   HAL_CAN_AddTxMessage(hcan, pHeader, pData, &mailbox);
	}
}

// 达妙电机位置模式的帧id是0x100+CAN_ID 即这里的CAN_TARGET_ID
// 达妙电机速度模式的帧id是0x200+CAN_ID
void Motor_3519_Speed(CAN_HandleTypeDef *hcan, float Speed, uint16_t CAN_Target_ID)
{
    uint32_t send_mail_box;
    Tx_DM3519_message.StdId = CAN_Target_ID;
    Tx_DM3519_message.IDE = CAN_ID_STD;
    Tx_DM3519_message.RTR = CAN_RTR_DATA;
    Tx_DM3519_message.DLC = 0x04;
    memcpy(DM3519_send_data, &Speed, 4);
    CAN_Tx_Retry(hcan, &Tx_DM3519_message, DM3519_send_data);
}

void Motor_3519_Position(CAN_HandleTypeDef *hcan, float Position, float Speed, uint16_t CAN_Target_ID)
{
    uint32_t send_mail_box;
    Tx_DM3519_message.StdId = CAN_Target_ID;
    Tx_DM3519_message.IDE = CAN_ID_STD;
    Tx_DM3519_message.RTR = CAN_RTR_DATA;
    Tx_DM3519_message.DLC = 0x08;
    memcpy(&DM3519_send_data[0], &Position, 4);
    memcpy(&DM3519_send_data[4], &Speed, 4);
    CAN_Tx_Retry(hcan, &Tx_DM3519_message, DM3519_send_data);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Rx1_header;
    uint8_t Rx1_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1_header, Rx1_data) == HAL_OK)
    {
        if (hcan == &hcan1)
        {
            switch (Rx1_header.StdId)
            {
            case CAN_3508_M1_ID:
                Get_Motor_Measure(&motor_chassis[0], Rx1_data); // 解析电机1数据
                UpdateEncoder(&motor_chassis[0]);               // 更新电机1累计角度
                break;
            case CAN_3508_M2_ID:
                Get_Motor_Measure(&motor_chassis[1], Rx1_data); // 解析电机2数据
                UpdateEncoder(&motor_chassis[1]);               // 更新电机2累计角度
                break;
            case CAN_3508_M3_ID:
                Get_Motor_Measure(&motor_chassis[2], Rx1_data); // 解析电机3数据
                UpdateEncoder(&motor_chassis[2]);               // 更新电机3累计角度
                break;
            case CAN_3508_M4_ID:
                Get_Motor_Measure(&motor_chassis[3], Rx1_data); // 解析电机4数据
                UpdateEncoder(&motor_chassis[3]);               // 更新电机4累计角度
                break;
            case CAN_3508_M5_ID:
                Get_Motor_Measure(&motor_chassis[4], Rx1_data); // 解析电机5数据
                UpdateEncoder(&motor_chassis[4]);               // 更新电机5累计角度
                break;
            case CAN_3508_M6_ID:
                Get_Motor_Measure(&motor_chassis[5], Rx1_data); // 解析电机6数据
                UpdateEncoder(&motor_chassis[5]);               // 更新电机6累计角度
                break;
            case CAN_3508_M7_ID:
                Get_Motor_Measure(&motor_chassis[6], Rx1_data); // 解析电机7数据
                UpdateEncoder(&motor_chassis[6]);               // 更新电机7累计角度
                break;
            case CAN_3508_M8_ID:
                Get_Motor_Measure(&motor_chassis[7], Rx1_data); // 解析电机8数据
                UpdateEncoder(&motor_chassis[7]);               // 更新电机8累计角度
                break;
            case 0x141:
                Get_Motor_K_TECH_Measure(&motor_k_tech1, Rx1_data); // 解析瓴控电机数据
                break;
            case 0x15:
                Get_Motor_3519_Measure(&motor_3519_Buffer, Rx1_data);
                break;
            }
            switch (Rx1_header.ExtId)
            {
            case CAN_RS02_BACK_EXTID:
                Get_Motor_RS02_Measure(&motor_rs02, Rx1_data); // 解析灵足电机
                break;
            }
        }
    }
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Rx2_header;
    uint8_t Rx2_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Rx2_header, Rx2_data) == HAL_OK)
    {
        if (hcan == &hcan2)
        {
            switch (Rx2_header.StdId)
            {
            case CAN_3508_M1_ID:
                Get_Motor_Measure(&motor_chassis[0], Rx2_data); // 解析电机1数据
                UpdateEncoder(&motor_chassis[0]);               // 更新电机1累计角度
                break;
            case CAN_3508_M2_ID:
                Get_Motor_Measure(&motor_chassis[1], Rx2_data); // 解析电机2数据
                UpdateEncoder(&motor_chassis[1]);               // 更新电机2累计角度
                break;
            case CAN_3508_M3_ID:
                Get_Motor_Measure(&motor_chassis[2], Rx2_data); // 解析电机3数据
                UpdateEncoder(&motor_chassis[2]);               // 更新电机3累计角度
                break;
            case CAN_3508_M4_ID:
                Get_Motor_Measure(&motor_chassis[3], Rx2_data); // 解析电机4数据
                UpdateEncoder(&motor_chassis[3]);               // 更新电机4累计角度
                break;
            case CAN_3508_M5_ID:
                Get_Motor_Measure(&motor_chassis[4], Rx2_data); // 解析电机5数据
                UpdateEncoder(&motor_chassis[4]);               // 更新电机5累计角度
                break;
            case CAN_3508_M6_ID:
                Get_Motor_Measure(&motor_chassis[5], Rx2_data); // 解析电机6数据
                UpdateEncoder(&motor_chassis[5]);               // 更新电机6累计角度
                break;
            case CAN_3508_M7_ID:
                Get_Motor_Measure(&motor_chassis[6], Rx2_data); // 解析电机7数据
                UpdateEncoder(&motor_chassis[6]);               // 更新电机7累计角度
                break;
            case CAN_3508_M8_ID:
                Get_Motor_Measure(&motor_chassis[7], Rx2_data); // 解析电机8数据
                UpdateEncoder(&motor_chassis[7]);               // 更新电机8累计角度
                break;
            }
        }
    }
}
