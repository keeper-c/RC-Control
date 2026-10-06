#include "main.h"
M3508 C1Motor_1, C1Motor_2, C1Motor_3, C1Motor_4, C1Motor_5, C1Motor_6, C1Motor_7, C1Motor_8;
M3508 C2Motor_1, C2Motor_2, C2Motor_3, C2Motor_4, C2Motor_5, C2Motor_6, C2Motor_7, C2Motor_8;

int16_t out1, out2, out3, out4, out5, out6, out7, out8;
int16_t put1, put2, put3, put4, put5, put6, put7, put8;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
CAN_FilterTypeDef canFilterConfig1;
CAN_FilterTypeDef canFilterConfig2;
void CAN_Filter_Init(void)
{
//    canFilterConfig1.FilterBank = 0; // CAN1和CAN2共享FiFo0
//    canFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
//    canFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
//    canFilterConfig1.FilterIdHigh = 0x0000;
//    canFilterConfig1.FilterIdLow = 0x0000;
//    canFilterConfig1.FilterMaskIdHigh = 0x0000;
//    canFilterConfig1.FilterMaskIdLow = 0x0000;
//    canFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    canFilterConfig1.FilterActivation = ENABLE;

//    canFilterConfig2.FilterBank = 14;
//    canFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
//    canFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
//    canFilterConfig2.FilterIdHigh = 0x0000;
//    canFilterConfig2.FilterIdLow = 0x0000;
//    canFilterConfig2.FilterMaskIdHigh = 0x0000;
//    canFilterConfig2.FilterMaskIdLow = 0x0000;
//    canFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//    canFilterConfig2.FilterActivation = ENABLE;

//    HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig1);
//    HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig2);

//    HAL_CAN_Start(&hcan1);
//    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    // 现版本can1，can2共用FiFo0 ,望改进

    CAN_FilterTypeDef canFilterConfig1;
    canFilterConfig1.FilterBank = 0;
    canFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig1.FilterIdHigh = 0x0000;
    canFilterConfig1.FilterIdLow = 0x0000;
    canFilterConfig1.FilterMaskIdHigh = 0x0000;
    canFilterConfig1.FilterMaskIdLow = 0x0000;
    canFilterConfig1.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canFilterConfig1.FilterActivation = ENABLE;
    canFilterConfig1.SlaveStartFilterBank = 14;  // 从此号开始为CAN2使用（重要）

    CAN_FilterTypeDef canFilterConfig2;
    canFilterConfig2.FilterBank = 14;
    canFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig2.FilterIdHigh = 0x0000;
    canFilterConfig2.FilterIdLow = 0x0000;
    canFilterConfig2.FilterMaskIdHigh = 0x0000;
    canFilterConfig2.FilterMaskIdLow = 0x0000;
    canFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;  // 改为 FIFO1
    canFilterConfig2.FilterActivation = ENABLE;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig1);
    HAL_CAN_ConfigFilter(&hcan2, &canFilterConfig2);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void CAN_TransmitMessage_StdID(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailbox;
    txHeader.StdId = StdId;
    txHeader.ExtId = 0x01;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.IDE = CAN_ID_STD;
    txHeader.DLC = length;
    txHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(hcan, &txHeader, data, &mailbox);
}
void CAN_TransmitMessage_ExtID(CAN_HandleTypeDef *hcan, uint32_t ExtID, uint8_t *pData, uint8_t length)
{
    if (length > 8)
        return; // CAN 数据帧最多 8 字节

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.ExtId = ExtID;
    TxHeader.IDE = CAN_ID_EXT; // 使用扩展帧 (29-bit ID)
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, pData, &TxMailbox) != HAL_OK)
    {
        return;
    }
}

void motor5065(uint32_t ExtID, int32_t vec)
{
    uint8_t txData[4];
    txData[0] = (vec >> 24) & 0xFF; // 高 8 位
    txData[1] = (vec >> 16) & 0xFF; // 次高 8 位
    txData[2] = (vec >> 8) & 0xFF;  // 次低 8 位
    txData[3] = vec & 0xFF;         // 低 8 位
    CAN_TransmitMessage_ExtID(&hcan2, ExtID, txData, 4);
}

void motor_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t txData[8];
    txData[0] = motor1 >> 8;
    txData[1] = motor1;
    txData[2] = motor2 >> 8;
    txData[3] = motor2;
    txData[4] = motor3 >> 8;
    txData[5] = motor3;
    txData[6] = motor4 >> 8;
    txData[7] = motor4;
    CAN_TransmitMessage_StdID(&hcan1, 0x200, txData, 8);
}

void up_can1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t txData[8];
    txData[0] = motor1 >> 8;
    txData[1] = motor1;
    txData[2] = motor2 >> 8;
    txData[3] = motor2;
    txData[4] = motor3 >> 8;
    txData[5] = motor3;
    txData[6] = motor4 >> 8;
    txData[7] = motor4;
    CAN_TransmitMessage_StdID(&hcan1, 0x1FF, txData, 8);
}

void motor_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t txData[8];
    txData[0] = motor1 >> 8;
    txData[1] = motor1;
    txData[2] = motor2 >> 8;
    txData[3] = motor2;
    txData[4] = motor3 >> 8;
    txData[5] = motor3;
    txData[6] = motor4 >> 8;
    txData[7] = motor4;
    CAN_TransmitMessage_StdID(&hcan2, 0x200, txData, 8);
}

void up_can2(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint8_t txData[8];
    txData[0] = motor1 >> 8;
    txData[1] = motor1;
    txData[2] = motor2 >> 8;
    txData[3] = motor2;
    txData[4] = motor3 >> 8;
    txData[5] = motor3;
    txData[6] = motor4 >> 8;
    txData[7] = motor4;
    CAN_TransmitMessage_StdID(&hcan2, 0x1FF, txData, 8);
}

// 绝对角度获取
void get_absolute_angle(M3508 *motor)
{
    uint16_t current_encoder_value = motor->encoder;
    int16_t delta = current_encoder_value - motor->last_encoder_value;
    if (delta > 4096)
    {
        delta -= 8192;
    }
    else if (delta < -4096)
    {
        delta += 8192;
    }
    motor->cumulative_angle += delta;
    motor->last_encoder_value = current_encoder_value;
}

// can1 消息接收
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        if (hcan == &hcan1)
        {
            switch (rxHeader.StdId)
            {
            case 0x201:
                C1Motor_1.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_1.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_1.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_1.temper = rxData[6];
                break;
            case 0x202:
                C1Motor_2.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_2.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_2.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_2.temper = rxData[6];
                break;
            case 0x203:
                C1Motor_3.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_3.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_3.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_3.temper = rxData[6];
                break;
            case 0x204:
                C1Motor_4.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_4.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_4.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_4.temper = rxData[6];
                break;
            case 0x205:
                C1Motor_5.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_5.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_5.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_5.temper = rxData[6];
                break;
            case 0x206:
                C1Motor_6.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_6.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_6.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_6.temper = rxData[6];
                break;
            case 0x207:
                C1Motor_7.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_7.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_7.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_7.temper = rxData[6];
                break;
            case 0x208:
                C1Motor_8.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                C1Motor_8.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                C1Motor_8.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                C1Motor_8.temper = rxData[6];
                break;
            }
        }
//       if(hcan == &hcan2)
//       {
//            switch (rxHeader.StdId)
//                    {
//                      case 0x201: C2Motor_1.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                    C2Motor_1.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                            break;
//                        case 0x202: C2Motor_2.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                    C2Motor_2.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                            break;
//                       case 0x203: C2Motor_3.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                C2Motor_3.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                           break;
//                       case 0x204: C2Motor_4.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                    C2Motor_4.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                            break;
//                       case 0x205: C2Motor_5.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                   C2Motor_5.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                          break;
//                     case 0x206: C2Motor_6.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                                   C2Motor_6.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                    break;
//               case 0x207: C2Motor_7.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                           C2Motor_7.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                    break;
//                case 0x208: C2Motor_8.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
//                            C2Motor_8.speed = ((uint16_t)rxData[2] << 8) + rxData[3];

//                    break;
//          }
//        }
       }
}
//can2接受 有点问题
 void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxHeader, rxData) == HAL_OK)
    {
        if(hcan == &hcan2)
        {
            switch (rxHeader.StdId)
            {
                case 0x201: C2Motor_1.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_1.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_1.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_1.temper = rxData[6];
                    break;
                case 0x202: C2Motor_2.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_2.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_2.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_2.temper = rxData[6];
                    break;
                case 0x203: C2Motor_3.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_3.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_3.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_3.temper = rxData[6];
                    break;
                case 0x204: C2Motor_4.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_4.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_4.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_4.temper = rxData[6];
                    break;
                case 0x205: C2Motor_5.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_5.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_5.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_5.temper = rxData[6];
                    break;
                case 0x206: C2Motor_6.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_6.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_6.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_6.temper = rxData[6];
                    break;
                case 0x207: C2Motor_7.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_7.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_7.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_7.temper = rxData[6];
                    break;
                case 0x208: C2Motor_8.encoder = ((uint16_t)rxData[0] << 8) + rxData[1];
                            C2Motor_8.speed = ((uint16_t)rxData[2] << 8) + rxData[3];
                            C2Motor_8.iq = ((uint16_t)rxData[4] << 8) + rxData[5];
                            C2Motor_8.temper = rxData[6];
                    break;
            }
        }
    }
}
