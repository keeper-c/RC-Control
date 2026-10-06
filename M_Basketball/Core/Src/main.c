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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "tim.h"
#include "drv_DM_Control.h"
#include "drv_can.h"
#include "drv_DJ_Control.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "alg_pid.h"
#include "Chassis.h"
#include "Serial_AutoPID.h"
#include "sbus.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

Chassis_t p_Chassis1;
More_Motor_t p_More_Motor1;
DJ_Motor_t p_DJMotor;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

void DM_RX_Callback(Struct_CAN_Rx_Buffer *pRxBuffer)
{
   switch(pRxBuffer->Header.StdId)
   {/*
       case 0x11:
       {
          DM_RX_Switch(&p_Chassis1.DM_Motor[0], pRxBuffer);
          break;
       }
       case 0x12:
       {
          DM_RX_Switch(&p_Chassis1.DM_Motor[1], pRxBuffer);
          break;
       }
       case 0x13:
       {
          DM_RX_Switch(&p_Chassis1.DM_Motor[2], pRxBuffer);
          break;
       }
       case 0x14:
       {
          DM_RX_Switch(&p_Chassis1.DM_Motor[3], pRxBuffer);
          break;
       }
       */
       case 0x201:
       {
          DJ_Rx_Switch(&p_DJMotor, pRxBuffer); 
          break;
       }
       case 0x202:
       {
         DJ_Rx_Switch(&p_Chassis1.DJMotor[1], pRxBuffer);
          break;
       }
       case 0x203:
       {
          DJ_Rx_Switch(&p_Chassis1.DJMotor[2], pRxBuffer);
          break;
       }
       case 0x204:
       {
          DJ_Rx_Switch(&p_Chassis1.DJMotor[3], pRxBuffer);
          break;
       }
       case 0x205:
       {
          break;
       }
       case 0x206:
       {
          break;
       }  
       case 0x207:
       {
          break;
       }  
       case 0x208:
       {
          break;
       }
       default:
       {
          break;
       }   
   }
}

void JP_Control(uint8_t State)
{
  if(State == 0)
  {
    HAL_GPIO_WritePin(JP_GPIO_Port, JP_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(JP_GPIO_Port, JP_Pin, GPIO_PIN_SET);
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  CAN_Init(&hcan1, DM_RX_Callback);
  CAN_Init(&hcan2, DM_RX_Callback);
  CAN_Filter_Mask_Config(&hcan1, CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x00, 0x00);
  CAN_Filter_Mask_Config(&hcan2, CAN_FIFO_0 | CAN_STDID | CAN_DATA_TYPE, 0x00, 0x00);
  HAL_TIM_Base_Start_IT(&htim1);

  SBUS_Init(&sbus_data, &huart1);

  uint16_t IDs1[4] = {0x1, 0x2, 0x3, 0x4};
  uint16_t IDs2[MORE_MOTOR_NUM] = {0x5, 0x6, 0x7, 0x8, 0x9};
  float Target[MORE_MOTOR_NUM] = {0.0f, 0.0f, 0.0f, 0.0f,  0.0f};
  
  Chassis_Init(&p_Chassis1, Chassis_Motor_DM, Chassis_Mode_Mecanum, &hcan1, IDs1);
  More_Motor_Init(&p_More_Motor1, Chassis_Motor_DM, &hcan2, IDs2);

  //Chassis_Set_Target(&p_Chassis1, 0.0f, 0.0f, 0.0f);


/*  
  Serial_AutoPID_Init(&g_AutoPID, &huart2, 0xAA);
  DJ_Motor_Init(&p_DJMotor, &hcan1, 0x01, DJ_MOTOR_MODE_SPEED, MOTOR_REDUCTION_RATIO, DJ_Send_Data_0x200);
  DJ_Set_Target(&p_DJMotor, 0.0f);
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(sbus_data.ch[5] < 100 && sbus_data.ch[5] >-100)
	  {
		  Target[0] = 0.0;
		  Target[1] = 0.0;
	  }
	  else if(sbus_data.ch[5] < -100 && sbus_data.ch[5] > -1000)
	  {
		  Target[0] = -6.0;
		  Target[1] = 6.0;
	  }
    
    if(sbus_data.ch[6] <  100 && sbus_data.ch[6] > -100)
    {
      Target[2] = 0.0;
      Target[3] = 0.0;
    }
    else if(sbus_data.ch[6] < -100 && sbus_data.ch[6] > -1000)
    {
      Target[2] = -150.0;
      Target[3] = 150.0;
    }

    if(sbus_data.ch[4] < 100)
    {
      JP_Control(0);
    }
    else if(sbus_data.ch[4] > 100)
    {
      JP_Control(1);
    }
    
    if(sbus_data.ch[7] < 100 )
    {
      Target[4] = 0.0;
    }
    else if(sbus_data.ch[7] > 100)
    {
      Target[4] = 60.0;
    }
  

    More_Motor_Set_Target(&p_More_Motor1, Target);
    Chassis_Set_Target(&p_Chassis1, map_sym_dead(sbus_data.ch[3], 50, 50.0f), map_sym_dead(sbus_data.ch[2], 50, 50.0f), map_sym_dead(sbus_data.ch[0], 50, 50.0f));
    More_Motor_Control(&p_More_Motor1);
    Chassis_Control(&p_Chassis1);
    HAL_Delay(1);

    //printf("ch0: %d, ch1: %d, ch2: %d, ch3: %d, ch4: %d, ch5: %d, ch6: %d, ch17: %d, ch18: %d\n", sbus_data.ch[0], sbus_data.ch[1], sbus_data.ch[2], sbus_data.ch[3], sbus_data.ch[4], sbus_data.ch[5], sbus_data.ch[6], sbus_data.ch17, sbus_data.ch18);
    //HAL_UART_Transmit(&huart2, sbus_raw_buf, SBUS_LEN, HAL_MAX_DELAY);
/*
    if(g_AutoPID.pid_ready)
    {
      p_DJMotor.speed_pid.Kp = g_AutoPID.Kp;
      p_DJMotor.speed_pid.Ki = g_AutoPID.Ki;
      p_DJMotor.speed_pid.Kd = g_AutoPID.Kd;
      g_AutoPID.pid_ready = 0;
    }
    if(g_AutoPID.target_ready)
    {
      p_DJMotor.speed_pid.target = g_AutoPID.target;
      g_AutoPID.target_ready = 0;
    }
    DJ_Motor_Control(&p_DJMotor);
    Serial_AutoPID_Send(&g_AutoPID, &p_DJMotor);
    HAL_Delay(1);
*/
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/**
 * @brief 定时器周期溢出回调(HAL 弱函数重写)
 * @note TIM1 配置为 1kHz(1ms), 作为电机控制唯一 tick 来源:
 *         1. 生成正弦速度目标 + 加速度前馈(相位累加器, 与控制环同周期)
 *         2. 速度环 PI 运算 -> 更新电流给定
 *         3. 发送 C620 电流帧(持续发送, 喂电调看门狗)
 *       回调内禁止调用 HAL_Delay / 阻塞式串口发送。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
      //Chassis_Control(&p_Chassis);
    }
}


/**
 * @brief printf重定向到UART2 (Keil MDK-ARM)
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
