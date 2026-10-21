/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "portable.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_SYSVIEW_FreeRTOS.h"
#include "robot_param.h"
#include "Telemetry_Task.h"
#include "vision_task.h"
#include "slam_task.h"

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
osThreadId Start_INS_TaskHandle;
uint32_t Start_INS_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_INS_TaskControlBlock;
osThreadId Start_Control_TaskHandle;
uint32_t Start_Control_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Control_TaskControlBlock;
osThreadId Start_CAN_TaskHandle;
uint32_t Start_CAN_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_CAN_TaskControlBlock;
osThreadId Start_Detect_TaskHandle;
uint32_t Start_Detect_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Detect_TaskControlBlock;
osThreadId Start_Telemetry_TaskHandle;
/* 遥测帧包含大量浮点格式化字段，保留更充足的独立栈，避免 snprintf
 * 在复杂配置下侵占其他控制任务的栈。 */
uint32_t Start_Telemetry_TaskBuffer[ 2048 ];
osStaticThreadDef_t Start_Telemetry_TaskControlBlock;
osThreadId Start_Slam_TaskHandle;
uint32_t Start_Slam_TaskBuffer[ 512 ];
osStaticThreadDef_t Start_Slam_TaskControlBlock;
#if (VISION_LINK_ENABLE != 0U) && (USB_CDC_MODE == USB_CDC_MODE_VISION)
osThreadId Start_Vision_TaskHandle;
uint32_t Start_Vision_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Vision_TaskControlBlock;
#endif
#if (CHASSIS_TYPE != CHASSIS_NONE)
osThreadId Start_Chassis_TaskHandle;
uint32_t Start_Chassis_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Chassis_TaskControlBlock;
#endif
#if (GIMBAL_TYPE != GIMBAL_NONE)
osThreadId Start_Gimbal_TaskHandle;
uint32_t Start_Gimbal_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Gimbal_TaskControlBlock;
#endif
#if (SHOOT_TYPE != SHOOT_NONE)
osThreadId Start_Shoot_TaskHandle;
uint32_t Start_Shoot_TaskBuffer[ 1024 ];
osStaticThreadDef_t Start_Shoot_TaskControlBlock;
#endif

volatile uint8_t FreeRTOS_MallocFailed;
volatile uint8_t FreeRTOS_StackOverflow;
volatile TaskHandle_t FreeRTOS_StackOverflowTask;
volatile const char *FreeRTOS_StackOverflowTaskName;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void INS_Task(void const * argument);
void Control_Task(void const * argument);
void CAN_Task(void const * argument);
void Detect_Task(void const * argument);
void Telemetry_Task(void const * argument);
void Slam_Task(void const * argument);
#if (VISION_LINK_ENABLE != 0U) && (USB_CDC_MODE == USB_CDC_MODE_VISION)
void Vision_Task(void const * argument);
#endif
#if (CHASSIS_TYPE != CHASSIS_NONE)
void Chassis_Task(void const * argument);
#endif
#if (GIMBAL_TYPE != GIMBAL_NONE)
void Gimbal_Task(void const * argument);
#endif
#if (SHOOT_TYPE != SHOOT_NONE)
void Shoot_Task(void const * argument);
#endif

extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of Start_INS_Task */
  osThreadStaticDef(Start_INS_Task, INS_Task, osPriorityHigh, 0, 1024, Start_INS_TaskBuffer, &Start_INS_TaskControlBlock);
  Start_INS_TaskHandle = osThreadCreate(osThread(Start_INS_Task), NULL);

  /* definition and creation of Start_Control_Task */
  osThreadStaticDef(Start_Control_Task, Control_Task, osPriorityAboveNormal, 0, 1024, Start_Control_TaskBuffer, &Start_Control_TaskControlBlock);
  Start_Control_TaskHandle = osThreadCreate(osThread(Start_Control_Task), NULL);

  /* definition and creation of Start_CAN_Task */
  osThreadStaticDef(Start_CAN_Task, CAN_Task, osPriorityNormal, 0, 1024, Start_CAN_TaskBuffer, &Start_CAN_TaskControlBlock);
  Start_CAN_TaskHandle = osThreadCreate(osThread(Start_CAN_Task), NULL);

  /* definition and creation of Start_Detect_Task */
  osThreadStaticDef(Start_Detect_Task, Detect_Task, osPriorityBelowNormal, 0, 1024, Start_Detect_TaskBuffer, &Start_Detect_TaskControlBlock);
  Start_Detect_TaskHandle = osThreadCreate(osThread(Start_Detect_Task), NULL);

  /* USB telemetry is deliberately lower priority than every control task. */
  osThreadStaticDef(Start_Telemetry_Task, Telemetry_Task, osPriorityLow, 0, 2048,
                    Start_Telemetry_TaskBuffer, &Start_Telemetry_TaskControlBlock);
  Start_Telemetry_TaskHandle = osThreadCreate(osThread(Start_Telemetry_Task), NULL);

  /* SLAM 只解析 UART7 数据并维护在线状态，不占用运动控制路径。 */
  osThreadStaticDef(Start_Slam_Task, Slam_Task, osPriorityBelowNormal, 0, 512,
                    Start_Slam_TaskBuffer, &Start_Slam_TaskControlBlock);
  Start_Slam_TaskHandle = osThreadCreate(osThread(Start_Slam_Task), NULL);

#if (VISION_LINK_ENABLE != 0U) && (USB_CDC_MODE == USB_CDC_MODE_VISION)
  /* 视觉任务只做协议收发与命令发布，不直接占用电机控制路径。 */
  osThreadStaticDef(Start_Vision_Task, Vision_Task, osPriorityAboveNormal, 0, 1024,
                    Start_Vision_TaskBuffer, &Start_Vision_TaskControlBlock);
  Start_Vision_TaskHandle = osThreadCreate(osThread(Start_Vision_Task), NULL);
#endif

  /* Chassis and gimbal own their respective motor command paths. */
#if (CHASSIS_TYPE != CHASSIS_NONE)
  osThreadStaticDef(Start_Chassis_Task, Chassis_Task, osPriorityAboveNormal, 0, 1024, Start_Chassis_TaskBuffer, &Start_Chassis_TaskControlBlock);
  Start_Chassis_TaskHandle = osThreadCreate(osThread(Start_Chassis_Task), NULL);
#endif
#if (GIMBAL_TYPE != GIMBAL_NONE)
  osThreadStaticDef(Start_Gimbal_Task, Gimbal_Task, osPriorityAboveNormal, 0, 1024, Start_Gimbal_TaskBuffer, &Start_Gimbal_TaskControlBlock);
  Start_Gimbal_TaskHandle = osThreadCreate(osThread(Start_Gimbal_Task), NULL);
#endif
#if (SHOOT_TYPE != SHOOT_NONE)
  osThreadStaticDef(Start_Shoot_Task, Shoot_Task, osPriorityAboveNormal, 0, 1024, Start_Shoot_TaskBuffer, &Start_Shoot_TaskControlBlock);
  Start_Shoot_TaskHandle = osThreadCreate(osThread(Start_Shoot_Task), NULL);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* These hooks make heap exhaustion and a task stack overrun observable instead
 * of letting either fault corrupt unrelated control state. Inspect the flags
 * and xPortGetMinimumEverFreeHeapSize() in the debugger/telemetry. */
void vApplicationMallocFailedHook(void)
{
  FreeRTOS_MallocFailed = 1U;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  FreeRTOS_StackOverflow = 1U;
  FreeRTOS_StackOverflowTask = xTask;
  FreeRTOS_StackOverflowTaskName = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

/* USER CODE BEGIN Header_INS_Task */
/**
  * @brief  Function implementing the StartINS thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the StartControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */
__weak void CAN_Task(void const * argument)
{
  /* USER CODE BEGIN CAN_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Task */
}

/* USER CODE BEGIN Header_Detect_Task */
/**
* @brief Function implementing the Start_Detect_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Detect_Task */
__weak void Detect_Task(void const * argument)
{
  /* USER CODE BEGIN Detect_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Detect_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
