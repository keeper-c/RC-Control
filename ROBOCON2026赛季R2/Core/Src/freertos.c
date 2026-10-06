/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE BEGIN Variables */
/* Definitions for mutex_Act */
osMutexId_t mutex_ActHandle;
const osMutexAttr_t mutex_Act_attributes = {
  .name = "mutex_Act"
};
osMutexId_t mutex_UpDownHandle;
const osMutexAttr_t mutex_UpDown_attributes = {
  .name = "mutex_UpDown"
};

/* USER CODE END Variables */
/* Definitions for StateFlag_Task */
osThreadId_t StateFlag_TaskHandle;
const osThreadAttr_t StateFlag_Task_attributes = {
  .name = "StateFlag_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MOVE_Task */
osThreadId_t MOVE_TaskHandle;
const osThreadAttr_t MOVE_Task_attributes = {
  .name = "MOVE_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Action_Task */
osThreadId_t Action_TaskHandle;
const osThreadAttr_t Action_Task_attributes = {
  .name = "Action_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sem_command */
osSemaphoreId_t sem_commandHandle;
const osSemaphoreAttr_t sem_command_attributes = {
  .name = "sem_command"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StateFlag_Function(void *argument);
extern void MOVE_Function(void *argument);
extern void Action_Function(void *argument);
extern void StartCommandTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  mutex_ActHandle = osMutexNew(&mutex_Act_attributes);
  mutex_UpDownHandle = osMutexNew(&mutex_UpDown_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of sem_command */
  sem_commandHandle = osSemaphoreNew(1, 0, &sem_command_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of StateFlag_Task */
   StateFlag_TaskHandle = osThreadNew(StateFlag_Function, NULL, &StateFlag_Task_attributes);

  /* creation of MOVE_Task */
  MOVE_TaskHandle = osThreadNew(MOVE_Function, NULL, &MOVE_Task_attributes);

  /* creation of Action_Task */
  Action_TaskHandle = osThreadNew(Action_Function, NULL, &Action_Task_attributes);

  /* creation of CommandTask */
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StateFlag_Function */
/**
* @brief Function implementing the StateFlag_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateFlag_Function */
__weak void StateFlag_Function(void *argument)
{
  /* USER CODE BEGIN StateFlag_Function */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StateFlag_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

