/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "robot_def.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "omni_UI.h"
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
osThreadId_t BuzzerHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for instask */
osThreadId_t instaskHandle;
const osThreadAttr_t instask_attributes = {
  .name = "instask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for CMD */
osThreadId_t CMDHandle;
const osThreadAttr_t CMD_attributes = {
  .name = "CMD",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for Gimbal */
osThreadId_t GimbalHandle;
const osThreadAttr_t Gimbal_attributes = {
  .name = "Gimbal",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Shoot */
osThreadId_t ShootHandle;
const osThreadAttr_t Shoot_attributes = {
  .name = "Shoot",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for motorControl */
osThreadId_t motorControlHandle;
const osThreadAttr_t motorControl_attributes = {
  .name = "motorControl",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for UIDraw */
osThreadId_t UIDrawHandle;
const osThreadAttr_t UIDraw_attributes = {
  .name = "UIDraw",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for Daemon */
osThreadId_t DaemonHandle;
const osThreadAttr_t Daemon_attributes = {
  .name = "Daemon",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void BuzzerTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartINSTASK(void *argument);
void _RobotCMDTask(void *argument);
void _GimbalTask(void *argument);
void _ChassisTask(void *argument);
void _ShootTask(void *argument);
void motorControlTask(void *argument);
void _UITask(void *argument);
void _DaemonTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
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
    /* creation of CMD */
  CMDHandle = osThreadNew(_RobotCMDTask, NULL, &CMD_attributes);  
    motorControlHandle = osThreadNew(motorControlTask, NULL, &motorControl_attributes); 
    /* creation of Shoot */
  ShootHandle = osThreadNew(_ShootTask, NULL, &Shoot_attributes);
  /* creation of Gimbal */

 GimbalHandle = osThreadNew(_GimbalTask, NULL, &Gimbal_attributes);
 #if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
  /* creation of instask */
    instaskHandle = osThreadNew(StartINSTASK, NULL, &instask_attributes);
#endif
DaemonHandle = osThreadNew(_DaemonTask, NULL, &Daemon_attributes);
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
  /* creation of Chassis */
  ChassisHandle = osThreadNew(_ChassisTask, NULL, &Chassis_attributes);
  /* creation of UIDraw */
  UIDrawHandle = osThreadNew(_UITask, NULL, &UIDraw_attributes);
#endif
  /* creation of motorControl */

  /* creation of Daemon */


  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  BuzzerHandle = osThreadNew(BuzzerTask, NULL, &buzzerTask_attributes);
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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
    UNUSED(argument);
    osThreadTerminate(defaultTaskHandle); // 避免空置和切换占用cpu
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartINSTASK */
/**
* @brief Function implementing the instask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartINSTASK */
__weak void StartINSTASK(void *argument)
{
  /* USER CODE BEGIN StartINSTASK */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartINSTASK */
}

/* USER CODE BEGIN Header__RobotCMDTask */
/**
* @brief Function implementing the CMD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__RobotCMDTask */
__weak void _RobotCMDTask(void *argument)
{
  /* USER CODE BEGIN _RobotCMDTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _RobotCMDTask */
}

/* USER CODE BEGIN Header__GimbalTask */
/**
* @brief Function implementing the Gimbal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__GimbalTask */
__weak void _GimbalTask(void *argument)
{
  /* USER CODE BEGIN _GimbalTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _GimbalTask */
}

/* USER CODE BEGIN Header__ChassisTask */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ChassisTask */
__weak void _ChassisTask(void *argument)
{
  /* USER CODE BEGIN _ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ChassisTask */
}

/* USER CODE BEGIN Header__ShootTask */
/**
* @brief Function implementing the Shoot thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__ShootTask */
__weak void _ShootTask(void *argument)
{
  /* USER CODE BEGIN _ShootTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _ShootTask */
}

/* USER CODE BEGIN Header_motorControlTask */
/**
* @brief Function implementing the motorControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motorControlTask */
__weak void motorControlTask(void *argument)
{
  /* USER CODE BEGIN motorControlTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END motorControlTask */
}

/* USER CODE BEGIN Header__UITask */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__UITask */
__weak void _UITask(void *argument)
{
  /* USER CODE BEGIN _UITask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _UITask */
}

/* USER CODE BEGIN Header__DaemonTask */
/**
* @brief Function implementing the Daemon thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__DaemonTask */
__weak void _DaemonTask(void *argument)
{
  /* USER CODE BEGIN _DaemonTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END _DaemonTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

