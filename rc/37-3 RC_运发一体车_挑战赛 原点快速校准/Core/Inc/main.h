/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include <math.h>
#include <stdio.h>

#include "can_bsp.h"
#include "remote_con.h"
#include "pid.h"
#include "drv_bsp.h"
#include "l1s_measure.h"
#include "disc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//#define oput1 TIM5->CCR3
extern int16_t oput1;
typedef struct{
    uint8_t data[5];    // 用于接收数据
    int16_t xy[2];      // 两轴值
    int16_t move;       // 最终驱动值
    uint8_t flag;       // 标志现在是否有接收到数据
}vision_para;

extern vision_para vision;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define qifa3_Pin GPIO_PIN_4
#define qifa3_GPIO_Port GPIOE
#define qifa2_Pin GPIO_PIN_5
#define qifa2_GPIO_Port GPIOE
#define guang_Pin GPIO_PIN_0
#define guang_GPIO_Port GPIOF
#define qifa1_Pin GPIO_PIN_1
#define qifa1_GPIO_Port GPIOF
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define KEY_EXTI_IRQn EXTI2_IRQn
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
