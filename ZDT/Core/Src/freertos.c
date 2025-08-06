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
extern void MotorReadTimerCallback(void const *argument); // <-- 保留此行
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
osTimerId motor_read_timerHandle;
/* USER CODE END Variables */
osThreadId StartCanTaskHandle;
osThreadId StartChassisTasHandle;
osThreadId StartINSTaskHandle;
osThreadId StartUserTaskHandle;
osThreadId StartRosTaskHandle;
osThreadId StartGimbalTaskHandle;
osSemaphoreId can_cmd_semHandle;
osSemaphoreId can_cmd_sem_yawHandle;
osSemaphoreId can_cmd_sem_pitchHandle;
osSemaphoreId can_cmd_mutexHandle; // 新增全局互斥信号量

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Can_Task(void const * argument);
void Chassis_Task(void const * argument);
void INS_Task(void const * argument);
void User_Task(void const * argument);
void Ros_Task(void const * argument);
void Gimbal_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
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

  /* Create the semaphores(s) */
  /* definition and creation of can_cmd_sem */
  osSemaphoreDef(can_cmd_sem);
  can_cmd_semHandle = osSemaphoreCreate(osSemaphore(can_cmd_sem), 1);

  osSemaphoreDef(can_cmd_sem_yaw);
  can_cmd_sem_yawHandle = osSemaphoreCreate(osSemaphore(can_cmd_sem_yaw), 1);

  osSemaphoreDef(can_cmd_sem_pitch);
  can_cmd_sem_pitchHandle = osSemaphoreCreate(osSemaphore(can_cmd_sem_pitch), 1);

  osSemaphoreDef(can_cmd_mutex); // 新增
  can_cmd_mutexHandle = osSemaphoreCreate(osSemaphore(can_cmd_mutex), 1); // 新增

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // // 创建并启动定时器，周期200ms
  // osTimerDef(motor_read_timer, MotorReadTimerCallback);
  // motor_read_timerHandle = osTimerCreate(osTimer(motor_read_timer), osTimerPeriodic, NULL);
  // osTimerStart(motor_read_timerHandle, 200);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartCanTask */
  osThreadDef(StartCanTask, Can_Task, osPriorityNormal, 0, 256);
  StartCanTaskHandle = osThreadCreate(osThread(StartCanTask), NULL);

  /* definition and creation of StartChassisTas */
  osThreadDef(StartChassisTas, Chassis_Task, osPriorityIdle, 0, 256);
  StartChassisTasHandle = osThreadCreate(osThread(StartChassisTas), NULL);

  /* definition and creation of StartINSTask */
  osThreadDef(StartINSTask, INS_Task, osPriorityIdle, 0, 256);
  StartINSTaskHandle = osThreadCreate(osThread(StartINSTask), NULL);

  /* definition and creation of StartUserTask */
  osThreadDef(StartUserTask, User_Task, osPriorityIdle, 0, 1024);
  StartUserTaskHandle = osThreadCreate(osThread(StartUserTask), NULL);

  /* definition and creation of StartRosTask */
  osThreadDef(StartRosTask, Ros_Task, osPriorityIdle, 0, 1024);
  StartRosTaskHandle = osThreadCreate(osThread(StartRosTask), NULL);

  /* definition and creation of StartGimbalTask */
  osThreadDef(StartGimbalTask, Gimbal_Task, osPriorityIdle, 0, 1024);
  StartGimbalTaskHandle = osThreadCreate(osThread(StartGimbalTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Can_Task */
/**
  * @brief  Function implementing the StartCanTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Can_Task */
__weak void Can_Task(void const * argument)
{
  /* USER CODE BEGIN Can_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Can_Task */
}

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the StartChassisTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the StartINSTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_User_Task */
/**
* @brief Function implementing the StartUserTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_User_Task */
__weak void User_Task(void const * argument)
{
  /* USER CODE BEGIN User_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END User_Task */
}

/* USER CODE BEGIN Header_Ros_Task */
/**
* @brief Function implementing the StartRosTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ros_Task */
__weak void Ros_Task(void const * argument)
{
  /* USER CODE BEGIN Ros_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ros_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the StartGimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
