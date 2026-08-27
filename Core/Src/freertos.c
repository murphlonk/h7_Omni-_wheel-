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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskPTZ02 */
osThreadId_t TaskPTZ02Handle;
const osThreadAttr_t TaskPTZ02_attributes = {
  .name = "TaskPTZ02",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskChassis03 */
osThreadId_t TaskChassis03Handle;
const osThreadAttr_t TaskChassis03_attributes = {
  .name = "TaskChassis03",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskCan04 */
osThreadId_t TaskCan04Handle;
const osThreadAttr_t TaskCan04_attributes = {
  .name = "TaskCan04",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskPowermeter0 */
osThreadId_t TaskPowermeter0Handle;
const osThreadAttr_t TaskPowermeter0_attributes = {
  .name = "TaskPowermeter0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskSwitch06 */
osThreadId_t TaskSwitch06Handle;
const osThreadAttr_t TaskSwitch06_attributes = {
  .name = "TaskSwitch06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskIMU07 */
osThreadId_t TaskIMU07Handle;
const osThreadAttr_t TaskIMU07_attributes = {
  .name = "TaskIMU07",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void PTZTask02(void *argument);
void ChassisTask03(void *argument);
void CanTask04(void *argument);
void PowermeterTask05(void *argument);
void SwtichTask06(void *argument);
void IMU_task(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TaskPTZ02 */
  TaskPTZ02Handle = osThreadNew(PTZTask02, NULL, &TaskPTZ02_attributes);

  /* creation of TaskChassis03 */
  TaskChassis03Handle = osThreadNew(ChassisTask03, NULL, &TaskChassis03_attributes);

  /* creation of TaskCan04 */
  TaskCan04Handle = osThreadNew(CanTask04, NULL, &TaskCan04_attributes);

  /* creation of TaskPowermeter0 */
  TaskPowermeter0Handle = osThreadNew(PowermeterTask05, NULL, &TaskPowermeter0_attributes);

  /* creation of TaskSwitch06 */
  TaskSwitch06Handle = osThreadNew(SwtichTask06, NULL, &TaskSwitch06_attributes);

  /* creation of TaskIMU07 */
  TaskIMU07Handle = osThreadNew(IMU_task, NULL, &TaskIMU07_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_PTZTask02 */
/**
* @brief Function implementing the TaskPTZ02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PTZTask02 */
__weak void PTZTask02(void *argument)
{
  /* USER CODE BEGIN PTZTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PTZTask02 */
}

/* USER CODE BEGIN Header_ChassisTask03 */
/**
* @brief Function implementing the TaskChassis03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask03 */
__weak void ChassisTask03(void *argument)
{
  /* USER CODE BEGIN ChassisTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisTask03 */
}

/* USER CODE BEGIN Header_CanTask04 */
/**
* @brief Function implementing the TaskCan04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CanTask04 */
__weak void CanTask04(void *argument)
{
  /* USER CODE BEGIN CanTask04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CanTask04 */
}

/* USER CODE BEGIN Header_PowermeterTask05 */
/**
* @brief Function implementing the TaskPowermeter0 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PowermeterTask05 */
__weak void PowermeterTask05(void *argument)
{
  /* USER CODE BEGIN PowermeterTask05 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PowermeterTask05 */
}

/* USER CODE BEGIN Header_SwtichTask06 */
/**
* @brief Function implementing the TaskSwitch06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SwtichTask06 */
__weak void SwtichTask06(void *argument)
{
  /* USER CODE BEGIN SwtichTask06 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SwtichTask06 */
}

/* USER CODE BEGIN Header_IMU_task */
/**
* @brief Function implementing the TaskIMU07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_task */
__weak void IMU_task(void *argument)
{
  /* USER CODE BEGIN IMU_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

