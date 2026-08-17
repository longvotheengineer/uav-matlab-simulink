/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS/tasks.h"
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

/* USER CODE END Variables */
/* Definitions for autopilot_task */
osThreadId_t autopilot_taskHandle;
const osThreadAttr_t autopilot_task_attributes = {
  .name = "autopilot_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for rx_task */
osThreadId_t rx_taskHandle;
const osThreadAttr_t rx_task_attributes = {
  .name = "rx_task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for system_io_mutex */
osMutexId_t system_io_mutexHandle;
const osMutexAttr_t system_io_mutex_attributes = {
  .name = "system_io_mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void start_autopilot_task(void *argument);
void start_rx_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of system_io_mutex */
  system_io_mutexHandle = osMutexNew(&system_io_mutex_attributes);

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
  /* creation of autopilot_task */
  autopilot_taskHandle = osThreadNew(start_autopilot_task, NULL, &autopilot_task_attributes);

  /* creation of rx_task */
  rx_taskHandle = osThreadNew(start_rx_task, NULL, &rx_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_start_autopilot_task */
/**
  * @brief  Function implementing the autopilot_task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_autopilot_task */
void start_autopilot_task(void *argument)
{
  /* USER CODE BEGIN start_autopilot_task */
  /* Infinite loop */
  autopilot_task(argument);
  /* USER CODE END start_autopilot_task */
}

/* USER CODE BEGIN Header_start_rx_task */
/**
* @brief Function implementing the rx_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_rx_task */
void start_rx_task(void *argument)
{
  /* USER CODE BEGIN start_rx_task */
  /* Infinite loop */
  rx_task(argument);
  /* USER CODE END start_rx_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

