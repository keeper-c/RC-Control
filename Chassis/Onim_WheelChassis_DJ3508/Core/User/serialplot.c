#include "serialplot.h"

#ifndef SERIALPLOT_MIN
#define SERIALPLOT_MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef SERIALPLOT_MAX
#define SERIALPLOT_MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

Struct_Serialplot Serialplot;

void Serialplot_Init(Struct_Serialplot *serialplot, UART_HandleTypeDef *huart,
                     uint8_t rx_variable_num, char **rx_variable_list,
                     Enum_Serialplot_Data_Type tx_data_type, uint8_t frame_header)
{
    serialplot->huart = huart;

    Struct_UART_Manage_Object *puart = NULL;

    switch ((uint32_t)huart->Instance)
    {
        case (uint32_t)USART2:
        {
            puart = &UART2_Struct;
            break;
        }
        case (uint32_t)USART3:
        {
            puart = &UART3_Struct;
            break;
        }
        default:
        {
            return;
        }
    }

    serialplot->UART_Rx_Data = puart->UART_Rx_Data;
    serialplot->UART_Tx_Data = puart->UART_Tx_Data;
    serialplot->UART_Rx_Variable_Num = rx_variable_num;
    serialplot->UART_Rx_Variable_List = rx_variable_list;
    serialplot->UART_Tx_Data_Type = tx_data_type;
    serialplot->Frame_Header = frame_header;

    serialplot->UART_Tx_Data[0] = frame_header;
    serialplot->Data_Number = 0;
    serialplot->Variable_Index = -1;
    serialplot->Variable_Value = 0.0;
}

int8_t Serialplot_Get_Variable_Index(Struct_Serialplot *serialplot)
{
    return serialplot->Variable_Index;
}

double Serialplot_Get_Variable_Value(Struct_Serialplot *serialplot)
{
    return serialplot->Variable_Value;
}

void Serialplot_Set_Data(Struct_Serialplot *serialplot, uint8_t number, ...)
{
    va_list data_ptr;
    uint8_t i;

    va_start(data_ptr, number);

    for (i = 0; i < SERIALPLOT_MAX_DATA_CHANNEL && i < number; i++)
    {
        serialplot->Data[i] = va_arg(data_ptr, void *);
    }

    va_end(data_ptr);

    serialplot->Data_Number = i;
}

static uint8_t Serialplot_Parse_Variable_Name(Struct_Serialplot *serialplot)
{
    char tmp_variable_name[SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    uint8_t i;
    uint8_t flag = 0;

    for (flag = 0; serialplot->UART_Rx_Data[flag] != '=' && serialplot->UART_Rx_Data[flag] != '\0'; flag++)
    {
        tmp_variable_name[flag] = serialplot->UART_Rx_Data[flag];
    }
    tmp_variable_name[flag] = '\0';

    for (i = 0; i < serialplot->UART_Rx_Variable_Num; i++)
    {
        char *var = (char *)((uint32_t)serialplot->UART_Rx_Variable_List +
                             SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH * i);

        if (strcmp(tmp_variable_name, var) == 0)
        {
            serialplot->Variable_Index = (int8_t)i;
            return flag + 1;
        }
    }

    serialplot->Variable_Index = -1;
    return flag + 1;
}

static void Serialplot_Parse_Variable_Value(Struct_Serialplot *serialplot, uint8_t flag)
{
    int8_t i;
    int8_t tmp_dot_position = -1;
    int8_t sign_coefficient = 1;
    double value = 0.0;
    double decimal_factor = 1.0;

    if (serialplot->Variable_Index == -1)
    {
        return;
    }

    if (serialplot->UART_Rx_Data[flag] == '-')
    {
        sign_coefficient = -1;
        flag++;
    }

    for (i = flag; serialplot->UART_Rx_Data[i] != '#' && serialplot->UART_Rx_Data[i] != '\0'; i++)
    {
        if (serialplot->UART_Rx_Data[i] == '.')
        {
            tmp_dot_position = i;
        }
        else if (serialplot->UART_Rx_Data[i] >= '0' && serialplot->UART_Rx_Data[i] <= '9')
        {
            value = value * 10.0 + (double)(serialplot->UART_Rx_Data[i] - '0');
            if (tmp_dot_position != -1)
            {
                decimal_factor *= 10.0;
            }
        }
    }

    if (tmp_dot_position != -1)
    {
        value /= decimal_factor;
    }

    serialplot->Variable_Value = value * sign_coefficient;
}

void Serialplot_Output(Struct_Serialplot *serialplot)
{
    uint8_t *tx_buf = serialplot->UART_Tx_Data;
    uint8_t i;

    memset(serialplot->UART_Tx_Data, 0, UART_TX_BUFFER_SIZE);
    serialplot->UART_Tx_Data[0] = serialplot->Frame_Header;

    switch (serialplot->UART_Tx_Data_Type)
    {
        case Serialplot_Data_Type_UINT8:
        case Serialplot_Data_Type_INT8:
        {
            for (i = 0; i < serialplot->Data_Number; i++)
            {
                uint8_t *p = (uint8_t *)serialplot->Data[i];
                tx_buf[1 + i] = *p;
            }
            break;
        }
        case Serialplot_Data_Type_UINT16:
        case Serialplot_Data_Type_INT16:
        {
            for (i = 0; i < serialplot->Data_Number; i++)
            {
                uint8_t *p = (uint8_t *)serialplot->Data[i];
                tx_buf[1 + 2 * i] = p[0];
                tx_buf[1 + 2 * i + 1] = p[1];
            }
            break;
        }
        case Serialplot_Data_Type_UINT32:
        case Serialplot_Data_Type_INT32:
        case Serialplot_Data_Type_FLOAT:
        {
            for (i = 0; i < serialplot->Data_Number; i++)
            {
                uint8_t *p = (uint8_t *)serialplot->Data[i];
                tx_buf[1 + 4 * i] = p[0];
                tx_buf[1 + 4 * i + 1] = p[1];
                tx_buf[1 + 4 * i + 2] = p[2];
                tx_buf[1 + 4 * i + 3] = p[3];
            }
            break;
        }
        case Serialplot_Data_Type_DOUBLE:
        {
            for (i = 0; i < serialplot->Data_Number; i++)
            {
                uint8_t *p = (uint8_t *)serialplot->Data[i];
                tx_buf[1 + 8 * i] = p[0];
                tx_buf[1 + 8 * i + 1] = p[1];
                tx_buf[1 + 8 * i + 2] = p[2];
                tx_buf[1 + 8 * i + 3] = p[3];
                tx_buf[1 + 8 * i + 4] = p[4];
                tx_buf[1 + 8 * i + 5] = p[5];
                tx_buf[1 + 8 * i + 6] = p[6];
                tx_buf[1 + 8 * i + 7] = p[7];
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void Serialplot_TIM_Callback(Struct_Serialplot *serialplot, uint16_t Size)
{
    Serialplot_Output(serialplot);
    TIM_UART_PeriodElapsedCallback(serialplot->huart, Size);
}

void Serialplot_UART_RxCpltCallback(Struct_Serialplot *serialplot, uint8_t *rx_data)
{
    uint8_t flag = Serialplot_Parse_Variable_Name(serialplot);
    Serialplot_Parse_Variable_Value(serialplot, flag);
}
