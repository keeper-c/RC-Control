#include "SerialPlot.h"
/**
 * @file serialplot.c
 * @brief 串口绘图协议实现文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 实现串口绘图协议,用于与上位机通信调参和数据可视化
 * 通信格式: "变量名=数值#"
 */

#include "serialplot.h"
#include <math.h>

/* 私有变量 ----------------------------------------------------------------*/

/**
 * @brief 数据类型字节大小查找表
 *
 * 通过 Enum_Serialplot_Data_Type 枚举值索引，快速获取对应数据类型的字节大小。
 * 枚举值与索引一一对应，使用 sizeof 确保平台无关性。
 */
static const uint8_t data_type_size[] = {
    sizeof(uint8_t),   // Serialplot_Data_Type_UINT8  (0)
    sizeof(int8_t),    // Serialplot_Data_Type_INT8   (1)
    sizeof(uint16_t),  // Serialplot_Data_Type_UINT16 (2)
    sizeof(int16_t),   // Serialplot_Data_Type_INT16  (3)
    sizeof(uint32_t),  // Serialplot_Data_Type_UINT32 (4)
    sizeof(int32_t),   // Serialplot_Data_Type_INT32  (5)
    sizeof(float),     // Serialplot_Data_Type_FLOAT  (6)
    sizeof(double)     // Serialplot_Data_Type_DOUBLE (7)
};

/* 函数定义 ----------------------------------------------------------------*/

/**
 * @brief 串口绘图初始化
 *
 * 根据 UART 实例自动绑定缓冲区，初始化配置参数：
 * - 接收：绑定变量名列表，等待上位机下发参数
 * - 发送：配置数据类型和帧头，等待数据打包发送
 *
 * 发送帧格式：帧头(1B) + 通道1(data_size) + 通道2(data_size) + ...
 *
 * @param serialplot 串口绘图结构体指针
 * @param huart UART 句柄指针，用于自动匹配 USART2/USART3
 * @param rx_variable_num 可接收的变量数量
 * @param rx_variable_list 变量名字符串数组，用于匹配上位机下发的变量名
 * @param tx_data_type 发送数据的类型，决定每个通道的字节数
 * @param frame_header 帧头标识，上位机通过帧头识别数据包
 */
void Serialplot_Init(Struct_Serialplot *serialplot, UART_HandleTypeDef *huart,
                     uint8_t rx_variable_num, char **rx_variable_list,
                     Enum_Serialplot_Data_Type tx_data_type, uint8_t frame_header)
{
    /* UART 实例与结构体映射表，新增串口只需添加一行 */
    static const struct {
        USART_TypeDef *Instance;
        Struct_UART_Manage_Object *puart;
    } uart_map[] = {{USART2, &UART2_Struct}, {USART3, &UART3_Struct}};

    /* 绑定配置参数 */
    serialplot->huart = huart;
    serialplot->UART_Rx_Variable_Num = rx_variable_num;
    serialplot->UART_Rx_Variable_List = rx_variable_list;
    serialplot->UART_Tx_Data_Type = tx_data_type;
    serialplot->Frame_Header = frame_header;

    /* 初始化状态 */
    serialplot->Data_Number = 0;
    serialplot->Variable_Index = -1;
    serialplot->Variable_Value = 0.0;

    /* 查找对应的 UART 管理对象，绑定收发缓冲区 */
    for (uint8_t i = 0; i < sizeof(uart_map) / sizeof(uart_map[0]); i++)
    {
        if (huart->Instance == uart_map[i].Instance)
        {
            serialplot->UART_Rx_Data = uart_map[i].puart->UART_Rx_Data;
            serialplot->UART_Tx_Data = uart_map[i].puart->UART_Tx_Data;
            serialplot->UART_Tx_Data[0] = frame_header; // 预设帧头
            return;
        }
    }
}

/**
 * @brief 获取当前解析到的变量索引
 *
 * 在 UART 接收回调中解析完成后调用，获取匹配到的变量编号。
 *
 * @param serialplot 串口绘图结构体指针
 * @return int8_t 变量索引（0~N-1），未匹配到则返回 -1
 */
int8_t Serialplot_Get_Variable_Index(Struct_Serialplot *serialplot)
{
    return serialplot->Variable_Index;
}

/**
 * @brief 获取当前解析到的变量值
 *
 * 在 UART 接收回调中解析完成后调用，获取下发的参数值。
 *
 * @param serialplot 串口绘图结构体指针
 * @return double 变量值
 */
double Serialplot_Get_Variable_Value(Struct_Serialplot *serialplot)
{
    return serialplot->Variable_Value;
}

/**
 * @brief 设置待发送数据的指针
 *
 * 通过可变参数传入多个数据指针，数据将在 Serialplot_Output 中
 * 按初始化时配置的数据类型打包发送。
 *
 * 使用示例：
 *   float speed = 12.5f, angle = 45.0f;
 *   Serialplot_Set_Data(&plot, 2, &speed, &angle);
 *
 * @param serialplot 串口绘图结构体指针
 * @param number 数据通道数量（不超过 SERIALPLOT_MAX_DATA_CHANNEL）
 * @param ... 可变参数，每个参数为指向数据的 void* 指针
 */
void Serialplot_Set_Data(Struct_Serialplot *serialplot, uint8_t number, ...)
{
    va_list data_ptr;
    va_start(data_ptr, number);

    /* 逐个读取数据指针，最多取 SERIALPLOT_MAX_DATA_CHANNEL 个 */
    for (uint8_t i = 0; i < SERIALPLOT_MAX_DATA_CHANNEL && i < number; i++)
    {
        serialplot->Data[i] = va_arg(data_ptr, void *);
    }

    va_end(data_ptr);

    /* 实际通道数取 min(number, 最大通道数) */
    serialplot->Data_Number = (uint8_t)fmin((double)number, (double)SERIALPLOT_MAX_DATA_CHANNEL);
}

/**
 * @brief 解析接收数据中的变量名
 *
 * 从 UART_Rx_Data 中提取 '=' 之前的字符串作为变量名，
 * 在 UART_Rx_Variable_List 中查找匹配项，匹配成功则设置 Variable_Index。
 *
 * 协议格式：变量名=数值#
 *
 * @param serialplot 串口绘图结构体指针
 * @return uint8_t '=' 后面第一个字符的索引（即数值起始位置）
 */
static uint8_t Serialplot_Parse_Variable_Name(Struct_Serialplot *serialplot)
{
    char tmp_name[SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH];
    uint8_t flag = 0;

    /* 提取 '=' 之前的变量名字符串 */
    while (serialplot->UART_Rx_Data[flag] != '=' && serialplot->UART_Rx_Data[flag] != '\0')
    {
        tmp_name[flag] = serialplot->UART_Rx_Data[flag];
        flag++;
    }
    tmp_name[flag] = '\0';

    /* 在变量名列表中查找匹配 */
    for (uint8_t i = 0; i < serialplot->UART_Rx_Variable_Num; i++)
    {
        char *var = serialplot->UART_Rx_Variable_List[i];
        if (strcmp(tmp_name, var) == 0)
        {
            serialplot->Variable_Index = i; // 匹配成功，记录索引
            return flag + 1;                // 返回 '=' 后的位置
        }
    }

    serialplot->Variable_Index = -1; // 未匹配
    return flag + 1;
}

/**
 * @brief 解析接收数据中的变量值
 *
 * 从指定位置开始解析数值字符串，支持：
 * - 负数（以 '-' 开头）
 * - 整数（如 "123"）
 * - 小数（如 "12.34"）
 *
 * 数值以 '#' 或 '\0' 结束。解析结果存入 Variable_Value。
 *
 * @param serialplot 串口绘图结构体指针
 * @param flag 数值起始位置（'=' 后面第一个字符的索引）
 */
static void Serialplot_Parse_Variable_Value(Struct_Serialplot *serialplot, uint8_t flag)
{
    /* 变量名未匹配，跳过解析 */
    if (serialplot->Variable_Index == -1)
        return;

    int8_t sign = 1;      // 符号位
    double value = 0.0;   // 累加值
    double decimal = 1.0; // 小数因子
    uint8_t has_dot = 0;  // 是否遇到小数点

    /* 处理负号 */
    if (serialplot->UART_Rx_Data[flag] == '-')
    {
        sign = -1;
        flag++;
    }

    /* 逐字符解析，遇到 '#' 或 '\0' 结束 */
    for (; serialplot->UART_Rx_Data[flag] != '#' && serialplot->UART_Rx_Data[flag] != '\0'; flag++)
    {
        if (serialplot->UART_Rx_Data[flag] == '.')
        {
            has_dot = 1; // 标记小数点
        }
        else if (serialplot->UART_Rx_Data[flag] >= '0' && serialplot->UART_Rx_Data[flag] <= '9')
        {
            value = value * 10.0 + (serialplot->UART_Rx_Data[flag] - '0');
            if (has_dot)
                decimal *= 10.0; // 小数点后每多一位，除数乘10
        }
    }

    /* 有小数点则除以小数因子，否则直接使用整数部分 */
    serialplot->Variable_Value = (has_dot ? value / decimal : value) * sign;
}

/**
 * @brief 数据打包到发送缓冲区
 *
 * 将 Data[] 数组中所有通道的数据按照 data_type_size[] 对应的字节大小，
 * 逐字节拷贝到 UART_Tx_Data 发送缓冲区中。
 *
 * 发送缓冲区格式：
 *   [0]            = 帧头
 *   [1  ~ 1+size]  = 通道1数据（小端序）
 *   [1+size ~ ...] = 通道2数据
 *   ...
 *
 * @param serialplot 串口绘图结构体指针
 */
void Serialplot_Output(Struct_Serialplot *serialplot)
{
    uint8_t *tx_buf = serialplot->UART_Tx_Data;
    uint8_t size = data_type_size[serialplot->UART_Tx_Data_Type]; // 每个通道的字节数

    /* 清空缓冲区并设置帧头 */
    memset(tx_buf, 0, UART_TX_BUFFER_SIZE);
    tx_buf[0] = serialplot->Frame_Header;

    /* 逐通道逐字节拷贝数据 */
    for (uint8_t i = 0; i < serialplot->Data_Number; i++)
    {
        uint8_t *p = (uint8_t *)serialplot->Data[i];
        uint8_t offset = 1 + size * i; // 当前通道在缓冲区中的起始位置
        for (uint8_t j = 0; j < size; j++)
        {
            tx_buf[offset + j] = p[j];
        }
    }
}

/**
 * @brief 定时器发送回调（打包 + DMA 发送，一步完成）
 *
 * 应在定时器中断中调用，自动完成数据打包和 DMA 发送。
 * 调用此函数后无需再手动调用 Serialplot_Output 或 UART_Send_Data。
 *
 * @param serialplot 串口绘图结构体指针
 */
void Serialplot_TIM_Callback(Struct_Serialplot *serialplot)
{
    Serialplot_Output(serialplot);
    uint8_t size = data_type_size[serialplot->UART_Tx_Data_Type];
    UART_Send_Data(serialplot->huart, serialplot->UART_Tx_Data,
                   1 + serialplot->Data_Number * size);
}

/**
 * @brief UART 接收完成回调
 *
 * 在 HAL_UARTEx_RxEventCallback 中调用，解析接收到的数据，
 * 提取变量名和变量值，存入 Variable_Index 和 Variable_Value。
 *
 * 解析流程：
 * 1. 先解析变量名 → 找到匹配的索引
 * 2. 再解析变量值 → 存入对应数值
 *
 * @param serialplot 串口绘图结构体指针
 * @param rx_data 接收数据缓冲区指针（未使用，保留兼容）
 */
void Serialplot_UART_RxCpltCallback(Struct_Serialplot *serialplot, uint8_t *rx_data)
{
    uint8_t flag = Serialplot_Parse_Variable_Name(serialplot);
    Serialplot_Parse_Variable_Value(serialplot, flag);
}