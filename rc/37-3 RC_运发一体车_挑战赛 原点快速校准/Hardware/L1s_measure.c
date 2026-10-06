/*
 * 距离数据的串口读取需采用串口空闲中断
 * 波特率115200
 * 正常测量命令返回格式"D="+"ascii浮点数距离值"+"m,"+"ascii回光量"+"#\r\n"
 * 停止测量命令返回格式"STO\r\nOK\r\n"
 */
#include "L1s_measure.h"
#include "usart.h"
#include "string.h"

#define ONE_MEASURE "iSM"
#define FAST_CONTINUE_MEASURE "iFACM" // 快速连续测量
#define CONTINUE_MEASURE "iACM"       // 连续测量
#define STOP_MEASURE "iHALT"          // 停止测量

// 测量一次仅返回一次数据
// 使用单次测量时不能太晃动，否则数据偏差较大
void L1s_one_measure(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit_IT(huart, (uint8_t *)ONE_MEASURE, sizeof(ONE_MEASURE));
}
// 快速连续测量
void L1s_fast_measure(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit_IT(huart, (uint8_t *)FAST_CONTINUE_MEASURE, sizeof(FAST_CONTINUE_MEASURE));
}
// 连续测量
void L1s_continue_measure(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit_IT(huart, (uint8_t *)CONTINUE_MEASURE, sizeof(CONTINUE_MEASURE));
}
// 停止测量
void L1s_stop_measure(UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit_IT(huart, (uint8_t *)STOP_MEASURE, sizeof(STOP_MEASURE));
}

/**
 * @brief  对于串口读取数据的解算
 * @note   ch为串口空闲中断所得到的数据
 * @param  放入串口读取的数据，ch长度建议位20
 * @retval 得到距离单位（米/M），如果距离大于40则数据错误可以忽略（自行加判断语句）（手动狗头）
 */
float L1s_distance_solving(uint8_t *ch)
{
    float result = 0.0;
    float fraction = 1.0;
    uint8_t decimalFound = 0;
    for (uint8_t i = 2; i < strlen((char *)ch); i++)
    {
        if (ch[i] == 'm')
            break;
        if (ch[i] == '.')
        {
            if (decimalFound)
                break;
            decimalFound = 1;
            fraction = 0.1f;
            continue;
        }
        if (ch[i] >= '0' && ch[i] <= '9')
        {
            int digit = ch[i] - '0';
            if (decimalFound)
            {
                result += digit * fraction;
                fraction *= 0.1f;
            }
            else
                result = result * 10.0f + digit;
        }
        else
            break;
    }
    return result;
}
