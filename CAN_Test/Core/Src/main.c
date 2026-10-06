/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_FILTER(x) ((x) << 3)

#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)
#define CAN_DATA_FRAME CAN_DATA_TYPE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  初始化CAN通信
  * @param  hcan: CAN句柄指针
  * @retval None
  */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);                                            /* 启动CAN控制器 */
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);            /* 使能CAN FIFO0消息挂起中断 */
  __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);            /* 使能CAN FIFO1消息挂起中断 */
}

/**
  * @brief  配置CAN过滤器（掩码模式）
  * @param  hcan: CAN句柄指针
  * @param  Object_Para: 过滤器配置参数（bit0:保留, bit1:模式错误标志, bit2:FIFO选择, bit3-7:过滤器编号）
  * @param  ID: 过滤器ID（11位标准ID）
  * @param  Mask: 过滤器掩码（11位）
  * @retval None
  */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask)
{
  CAN_FilterTypeDef can_filter_init_config;                       /* CAN过滤器初始化配置结构体 */

  if(Object_Para & 0x01)                                         /* 检查过滤器编号是否有效 */
  {
    return;                                                       /* 过滤器编号错误，直接返回 */
  }
  if((Object_Para & 0x02) >> 1)                                  /* 检查过滤器模式是否正确 */
  {
    return; /* 过滤器模式错误，直接返回 */
  }
  /* 设置过滤器ID */
  can_filter_init_config.FilterIdHigh = (ID & 0x7FF) << 5;       /* 将11位ID左移5位存入高16位 */
  can_filter_init_config.FilterIdLow = 0x0000;                   /* 低16位清零 */
  
  /* 设置过滤器掩码 */
  can_filter_init_config.FilterMaskIdHigh = (Mask & 0x7FF) << 5; /* 将11位掩码左移5位存入高16位 */
  can_filter_init_config.FilterMaskIdLow = 0x0000;               /* 低16位清零 */

  /* 配置过滤器参数 */
  can_filter_init_config.FilterBank = (Object_Para >> 3) & 0x1F; /* 从Object_Para提取过滤器编号(bit3-7) */
  can_filter_init_config.FilterMode = CAN_FILTERMODE_IDMASK;       /* 设置为掩码模式 */
  can_filter_init_config.FilterScale = CAN_FILTERSCALE_32BIT;     /* 设置为32位过滤器 */
  can_filter_init_config.FilterActivation = ENABLE;               /* 使能过滤器 */
  
  can_filter_init_config.SlaveStartFilterBank = 14;               /* 设置从机过滤器起始编号为14 */

  can_filter_init_config.FilterFIFOAssignment = (Object_Para >> 2) & 0x01; /* 分配过滤器到FIFO0或FIFO1 */

  HAL_CAN_ConfigFilter(hcan, &can_filter_init_config);            /* 应用过滤器配置 */
}

/**
  * @brief  发送CAN数据帧
  * @param  hcan: CAN句柄指针
  * @param  ID: 消息ID（11位标准ID）
  * @param  Data: 指向发送数据缓冲区的指针
  * @param  Length: 发送数据长度（0-8字节）
  * @retval HAL状态码（HAL_OK表示成功，其他表示失败）
  */
uint8_t CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef tx_header;                                  /* CAN发送消息头结构体 */
  uint32_t mailbox;                                              /* 存储分配的发送邮箱 */

  tx_header.StdId = ID;                                          /* 设置标准ID */
  tx_header.ExtId = 0;                                           /* 扩展ID清零（不使用） */
  tx_header.IDE = CAN_ID_STD;                                    /* 设置为标准ID格式 */
  tx_header.RTR = CAN_RTR_DATA;                                  /* 设置为数据帧 */
  tx_header.DLC = Length;                                         /* 设置数据长度 */

  return (HAL_CAN_AddTxMessage(hcan, &tx_header, Data, &mailbox)); /* 添加消息到发送邮箱并返回状态 */
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */


  CAN_Init(&hcan1);
  CAN_Filter_Mask_Config(&hcan1, CAN_FILTER(0) | CAN_FIFO_1 | CAN_STDID | CAN_DATA_FRAME, 0x123, 0x7FF);
  CAN_Send_Data(&hcan1, 0x123, (uint8_t *)"Hello", 5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  CAN FIFO1消息挂起回调函数
  * @note   当CAN FIFO1中有新消息时，HAL库会调用此函数
  * @param  hcan: CAN句柄指针
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;                                  /* CAN接收消息头结构体 */
  uint8_t data[8];                                                /* 接收数据缓冲区（最多8字节） */

  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, data);    /* 从FIFO1读取消息 */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
