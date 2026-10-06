/**
 * @file serialplot.h
 * @brief 串口绘图协议头文件
 * @version 2.0
 * @date 2026-05-05
 *
 * 定义串口绘图协议所需的数据结构、枚举类型、宏定义和函数声明
 */

#ifndef SERIALPLOT_H
#define SERIALPLOT_H

/* 头文件包含 ------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdarg.h"
#include "My_UART.h"

/* 宏定义 ----------------------------------------------------------------*/

#define SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH (16) /**< 接收变量名最大长度 */
#define SERIALPLOT_MAX_DATA_CHANNEL (12)                 /**< 最大数据通道数 */
/* 类型定义 ----------------------------------------------------------------*/

/**
 * @brief 串口绘图数据类型枚举
 */
typedef enum
{
    Serialplot_Data_Type_UINT8,   /**< 无符号8位整数 */
    Serialplot_Data_Type_INT8,    /**< 有符号8位整数 */
    Serialplot_Data_Type_UINT16,  /**< 无符号16位整数 */
    Serialplot_Data_Type_INT16,   /**< 有符号16位整数 */
    Serialplot_Data_Type_UINT32,  /**< 无符号32位整数 */
    Serialplot_Data_Type_INT32,   /**< 有符号32位整数 */
    Serialplot_Data_Type_FLOAT,   /**< 32位浮点数 */
    Serialplot_Data_Type_DOUBLE   /**< 64位双精度浮点数 */
} Enum_Serialplot_Data_Type;



/**
 * @brief 串口绘图结构体
 *
 * 管理串口绘图协议的配置参数、缓冲区、数据和状态
 */
typedef struct Serialplot
{
    UART_HandleTypeDef *huart;                     /**< UART句柄 */
    uint8_t *UART_Rx_Data;                         /**< 接收数据缓冲区 */
    uint8_t *UART_Tx_Data;                         /**< 发送数据缓冲区 */
    uint8_t UART_Rx_Variable_Num;                  /**< 接收变量数量 */
    char **UART_Rx_Variable_List;                 /**< 接收变量名列表 */
    Enum_Serialplot_Data_Type UART_Tx_Data_Type;  /**< 发送数据类型 */
    uint8_t Frame_Header;                          /**< 帧头 */
    void *Data[SERIALPLOT_MAX_DATA_CHANNEL];       /**< 数据指针数组 */
    uint8_t Data_Number;                           /**< 数据数量 */
    int8_t Variable_Index;                         /**< 变量索引 */
    double Variable_Value;                          /**< 变量值 */
} Struct_Serialplot;


extern Struct_Serialplot Serialplot;

/* 函数声明 ------------------------------------------------------------*/

/**
 * @brief 串口绘图初始化
 * @param serialplot 串口绘图结构体指针
 * @param huart UART句柄指针
 * @param rx_variable_num 接收变量数量
 * @param rx_variable_list 接收变量名列表
 * @param tx_data_type 发送数据类型
 * @param frame_header 帧头
 */
void Serialplot_Init(Struct_Serialplot *serialplot, UART_HandleTypeDef *huart,
                     uint8_t rx_variable_num, char **rx_variable_list,
                     Enum_Serialplot_Data_Type tx_data_type, uint8_t frame_header);

/**
 * @brief 获取变量索引
 * @param serialplot 串口绘图结构体指针
 * @return int8_t 变量索引,未找到返回-1
 */
int8_t Serialplot_Get_Variable_Index(Struct_Serialplot *serialplot);

/**
 * @brief 获取变量值
 * @param serialplot 串口绘图结构体指针
 * @return double 变量值
 */
double Serialplot_Get_Variable_Value(Struct_Serialplot *serialplot);

/**
 * @brief 设置待发送数据
 * @param serialplot 串口绘图结构体指针
 * @param number 数据数量
 * @param ... 可变参数,数据指针列表
 */
void Serialplot_Set_Data(Struct_Serialplot *serialplot, uint8_t number, ...);

/**
 * @brief 数据打包到发送缓冲区
 * @param serialplot 串口绘图结构体指针
 */
void Serialplot_Output(Struct_Serialplot *serialplot);

/**
 * @brief 定时器发送回调
 * @param serialplot 串口绘图结构体指针
 */
void Serialplot_TIM_Callback(Struct_Serialplot *serialplot, uint16_t Size);

/**
 * @brief UART接收完成回调
 * @param serialplot 串口绘图结构体指针
 * @param rx_data 接收数据指针
 */
void Serialplot_UART_RxCpltCallback(Struct_Serialplot *serialplot, uint8_t *rx_data);

#endif /* SERIALPLOT_H */
