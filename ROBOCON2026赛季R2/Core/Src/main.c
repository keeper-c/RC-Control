/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_bsp.h"
#include "pid.h"
#include "usart_send.h"
#include "CommandTask.h"
#include "power_bsp.h"
#include "rob_core.h"
#include "CommandTask.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  NVIC_SetPriorityGrouping(0);

  /* USER CODE BEGIN Init */
  //__HAL_RCC_PWR_CLK_ENABLE();
  //HAL_PWR_EnableBkUpAccess();
  //__HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  MX_UART8_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1);
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  RC_R2_Init();
  Can_Filter_Init();
  
#if 1 // ==== 阶段2：CAN + hcan2 电机 (如果阶段1正常，改为 #if 1) ====
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M1_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M2_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M3_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M4_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M5_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M6_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M7_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan2,CAN_3519_MODE_POSITION,CAN_3519_M8_ID);
  HAL_Delay(1);
#endif
  
#if 1 // ==== 阶段3：再加 hcan1 电机 (阶段2正常后改为 #if 1) ====
  Motor_3519_Init(&hcan1,CAN_3519_MODE_SPEED,CAN_3519_M5_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan1,CAN_3519_MODE_SPEED,CAN_3519_M6_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan1,CAN_3519_MODE_SPEED,CAN_3519_M7_ID);
  HAL_Delay(1);
  Motor_3519_Init(&hcan1,CAN_3519_MODE_SPEED,CAN_3519_M8_ID);
  HAL_Delay(1);
	
	//夹爪小电机
	Motor_3519_Init(&hcan1,CAN_3519_MODE_POSITION,CAN_3519_M1_ID);
  HAL_Delay(1);
#endif

  int   KP_F=22;    int    KP_T=15;  int    KP_T_auto=5;
  float KI_F=0;     float  KI_T=0;   float  KI_T_auto=0;
  int   KD_F=19;    int    KD_T=5;   int    KD_T_auto=0;

  Pid_Init(&pid_Follow_1,KP_F,KI_F,KD_F,5000,40,0,0);
  Pid_Init(&pid_Follow_2,KP_F,KI_F,KD_F,5000,40,0,0);
  Pid_Init(&pid_Follow_3,KP_F,KI_F,KD_F,5000,40,0,0);
  Pid_Init(&pid_Follow_4,KP_F,KI_F,KD_F,5000,40,0,0);

  Pid_Init(&pid_Turn_1,KP_T,KI_T,KD_T,5000,30,0,0);
  Pid_Init(&pid_Turn_2,KP_T,KI_T,KD_T,5000,30,0,0);
  Pid_Init(&pid_Turn_3,KP_T,KI_T,KD_T,5000,30,0,0);
  Pid_Init(&pid_Turn_4,KP_T,KI_T,KD_T,5000,30,0,0);

  Pid_Init(&pid_Auto_1,KP_T_auto,KI_T_auto,KD_T_auto,5000,10,0,0);
  Pid_Init(&pid_Auto_2,KP_T_auto,KI_T_auto,KD_T_auto,5000,10,0,0);
  Pid_Init(&pid_Auto_3,KP_T_auto,KI_T_auto,KD_T_auto,5000,10,0,0);
  Pid_Init(&pid_Auto_4,KP_T_auto,KI_T_auto,KD_T_auto,5000,10,0,0);
  
  Pid_Init(&pid_DJI_v2,3.1,0,2.1,5000,16000,0,0);
  Pid_Init(&pid_DJI_v3,3.1,0,2.1,5000,16000,0,0);

  Can_Notify_Enable();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint8_t RobotProtect = 0;
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1)
  { 
		static int32_t WatchDog_sick=0;
		//WTCHDOG-1s串口不搬运数据，机器人停止运动
		WatchDog_sick=osKernelGetTickCount()-watch_dog_tick;
		
		if(WatchDog_sick>1000)
		{
			RobotProtect=1;
			UART6_DMA_Restart();   // DMA卡死 → 复位串口DMA
		}
  }
  /* USER CODE END Callback 1 */
}

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
