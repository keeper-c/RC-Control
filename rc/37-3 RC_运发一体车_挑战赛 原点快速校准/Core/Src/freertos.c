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
#define Kuang_x 4.0f - 0.35f
#define Kuang_y 14.025f - 0.23f
float BuCang_x = 0, BuCang_y = 0, BuCang_z = 0;

float shot_angle, shot_speed;                                           // 发球的角度和速度
uint16_t para_speed = 2000;                                             // 速度，角度矫正常量
float shot_Pitch;                                                       // 发射机构俯仰，偏航实际PID控制入参
float distance_x, distance_y, distance_Gou, distance_Gu, distance_Xuan; // 单位m

uint8_t shot_flag = 0; // 投球时段标志位
float result7_stop;
int8_t up_down;          // 抬升机构
uint8_t guang_flag = 10; //  光电门标志位

extern PID_ Vc1_1;
extern PID_ Vc1_2;
extern PID_ Vc1_3;
extern PID_ Vc1_4;
extern PID_ Vc1_5;
extern PID_ Vc1_6;
extern PID_ Vc1_7;
extern PID_ Vc1_8;

extern PID_ Pc1_5;
extern PID_ Pc1_6;
extern PID_ Pc1_7;

extern PID_ vision_Pitch_pid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId PID_TaskHandle;
osThreadId Cloud_TaskHandle;
osThreadId LvBoQi_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void PID_Function(void const * argument);
void Cloud_Function(void const * argument);
void LvBoQi_Function(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* USER CODE END RTOS_MUTEX */

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
  /* definition and creation of PID_Task */
  osThreadDef(PID_Task, PID_Function, osPriorityNormal, 0, 256);
  PID_TaskHandle = osThreadCreate(osThread(PID_Task), NULL);

  /* definition and creation of Cloud_Task */
  osThreadDef(Cloud_Task, Cloud_Function, osPriorityBelowNormal, 0, 256);
  Cloud_TaskHandle = osThreadCreate(osThread(Cloud_Task), NULL);

  /* definition and creation of LvBoQi_Task */
  osThreadDef(LvBoQi_Task, LvBoQi_Function, osPriorityNormal, 0, 128);
  LvBoQi_TaskHandle = osThreadCreate(osThread(LvBoQi_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_PID_Function */
/**
 * @brief  Function implementing the PID_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_PID_Function */
void PID_Function(void const * argument)
{
  /* USER CODE BEGIN PID_Function */
    /* Infinite loop */
    for (;;)
    {
        float result5, result6, result7;
        float speed[4];
        driver_ddd(-Elrs.ch[4], Elrs.ch[3], Elrs.ch[1] / 1.6f, speed);

        out1 = PID_Calc(&Vc1_1, 500, C1Motor_1.speed ); 
        out2 = PID_Calc(&Vc1_2, 500, C1Motor_2.speed );
        out3 = PID_Calc(&Vc1_3, 500, C1Motor_3.speed );
        out4 = PID_Calc(&Vc1_4, 500, C1Motor_4.speed );

        result5 = PID_Calc(&Pc1_5, 0, C1Motor_5.cumulative_angle / 8192.f);
        out5 = PID_Calc(&Vc1_5, result5, C1Motor_5.speed / 19.0f);

        result6 = PID_Calc(&Pc1_6, 8192, C1Motor_6.cumulative_angle / 8192.0f);
        out6 = PID_Calc(&Vc1_6, result6, C1Motor_6.speed / 32.0f);

        if (distance_y < 6)
        {
            TIM5->CCR4 = 13000;
            result7 = PID_Calc(&Pc1_7, 0, C1Motor_7.cumulative_angle / 8192.0f);
        }
        else if (vision.flag == 0)
        {
            vision.xy[0] = 0;
            vision.xy[1] = 0;
            result7 = 0;
            TIM5->CCR4 = 13000;
        }
        else if (C1Motor_7.cumulative_angle / 8192.0f > 39 && vision.xy[0] > 0)
            result7 = PID_Calc(&Pc1_7, 40, C1Motor_7.cumulative_angle / 8192.0f);
        else if (C1Motor_7.cumulative_angle / 8192.0f < -60 && vision.xy[0] < 0)
            result7 = PID_Calc(&Pc1_7, -61, C1Motor_7.cumulative_angle / 8192.0f);
        else
            result7 = vision.xy[0] > 100 ? 100 : vision.xy[0] < -100 ? -100
                                                                     : vision.xy[0];
        if (shot_flag == 0)
            out7 = PID_Calc(&Vc1_7, result7, C1Motor_7.speed / 32.0f);
        else
        {
            result7 = PID_Calc(&Pc1_7, result7_stop, C1Motor_7.cumulative_angle / 8192.0f);
            out7 = PID_Calc(&Vc1_1, result7, C1Motor_7.speed / 32.0f);
        }
        TIM5->CCR4 += PID_Calc(&vision_Pitch_pid, 0, vision.xy[1]);
        TIM5->CCR4 = TIM5->CCR4 < 12000 ? 12000 : TIM5->CCR4 > 20000 ? 20000
                                                                     : TIM5->CCR4;
        osDelay(9);
    }
  /* USER CODE END PID_Function */
}

/* USER CODE BEGIN Header_Cloud_Function */
/**
 * @brief Function implementing the Cloud_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Cloud_Function */
void Cloud_Function(void const * argument)
{
  /* USER CODE BEGIN Cloud_Function */
    uint8_t bug_cao = 0, cnt = 0;
    /* Infinite loop */
    for (;;)
    {
        float x, y, angle_f;
        x = (pos_x - BuCang_x) / 1000.0f;
        y = (pos_y - BuCang_y) / 1000.0f;
        angle_f = deg2rad(BuCang_z);
        distance_x = x * cosf(angle_f) - y * sinf(angle_f);
        distance_y = x * sinf(angle_f) + y * cosf(angle_f);
        distance_Gu = sqrtf((distance_x - Kuang_x) * (distance_x - Kuang_x) + (distance_y - Kuang_y) * (distance_y - Kuang_y));

        if ((Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1) && bug_cao == 0) // 修bug用的
        {
            for (uint8_t i = 5; i < 10; i++)
                Elrs.ch_flag[i] = 0;
            bug_cao = 1;
        }

        if (Elrs.ch_flag[6] == 3)
        {
            BuCang_x = pos_x;
            BuCang_y = pos_y;
            BuCang_z = zangle;
            Elrs.ch_flag[5] = 0;
        }

        if(distance_y > 6) // 进入6m开始预加速
        {
            if(distance_Gu < 7)
            {
                HAL_TIM_Base_Start_IT(&htim12);
                solve_angle_speed(distance_Gu, &shot_angle, &shot_speed);
                shot_Pitch = ((75 - shot_angle) / 27.0f * 40.32f);
                oput1 = shot_speed * Forced_fitting(distance_Gu) * (Elrs.ch[10] / 10000.0f + 1.0f);
            }
            else
            {
                HAL_TIM_Base_Start_IT(&htim12);
                oput1 = 25000;
                shot_Pitch = 0;
            }
        }
        else
        {
            HAL_TIM_Base_Stop_IT(&htim12);
            shot_Pitch = 0;
        }

        if (Elrs.ch_flag[9] == 3) // 送球发射
        {
            shot_flag = 1;
            result7_stop = C1Motor_7.cumulative_angle / 8192.0f;
            up_down = 44;
            osDelay(600);
            shot_flag = 0;
            up_down = 0;
        }

        if (Elrs.ch[7] == 1 && (Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1)) // 运球，两颗
        {
            HAL_GPIO_WritePin(qifa3_GPIO_Port, qifa3_Pin, GPIO_PIN_SET);
            osDelay(1000); // 推送

            HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(qifa1_GPIO_Port, qifa1_Pin, GPIO_PIN_SET);
            osDelay(200);
            HAL_GPIO_WritePin(qifa1_GPIO_Port, qifa1_Pin, GPIO_PIN_RESET);
            osDelay(700);
            HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(qifa1_GPIO_Port, qifa1_Pin, GPIO_PIN_SET);
            osDelay(200);
            HAL_GPIO_WritePin(qifa1_GPIO_Port, qifa1_Pin, GPIO_PIN_RESET);
            osDelay(700);

            HAL_GPIO_WritePin(qifa3_GPIO_Port, qifa3_Pin, GPIO_PIN_RESET);
            osDelay(1000); // 推送(回)

            HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_SET);
            Elrs.ch_flag[8] = 0;
        }

        // 夹爪
        if (Elrs.ch[7] == 2 && (Elrs.ch_flag[8] == 3 || Elrs.ch_flag[8] == 1))
        {
            if (HAL_GPIO_ReadPin(qifa2_GPIO_Port, qifa2_Pin) == GPIO_PIN_SET)
                HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_RESET);
            else
                HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_SET);
            Elrs.ch_flag[8] = 0;
        }

        cnt++;
        if (cnt > 100)
        {
            cnt = 0;
            char lin[40];
            sprintf(lin, "x:%.2f,y:%.2f\ndis:%.2f", distance_x, distance_y, distance_Gu);
            HAL_UART_Transmit_IT(&huart2, (uint8_t *)lin, sizeof(lin));
        }
        osDelay(20);
    }
  /* USER CODE END Cloud_Function */
}

/* USER CODE BEGIN Header_LvBoQi_Function */
/**
 * @brief Function implementing the LvBoQi_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_LvBoQi_Function */
void LvBoQi_Function(void const * argument)
{
  /* USER CODE BEGIN LvBoQi_Function */
    uint8_t guang_last = 0, guang_now = 0, guang_count = 0;
    /* Infinite loop */
    for (;;)
    {
        guang_now = HAL_GPIO_ReadPin(guang_GPIO_Port, guang_Pin);
        if (guang_now != guang_last)
        {
            guang_count++;
        }
        if (guang_count > 10)
        {
            guang_count = 0;
            guang_flag = guang_now;
            guang_last = guang_now;
        }

        if (guang_flag == 0)
        {
            HAL_GPIO_WritePin(qifa2_GPIO_Port, qifa2_Pin, GPIO_PIN_RESET);
            guang_flag = 10;
        }
        osDelay(1);
    }
  /* USER CODE END LvBoQi_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
