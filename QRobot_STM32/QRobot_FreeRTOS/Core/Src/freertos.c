/* USER CODE BEGIN Header */
#include "motor.h"
#include "encoder.h"
#include "system.h"
#include "motion_model.h"
#include "pid.h"
#include "ps2.h"
#include "serial.h"
#include "mpu6050.h"
#include "oled.h"


// ---------------- 本文件为FreeRTOS的任务函数 -------------------------

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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
int PS2_key=0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId control_TaskHandle;
uint32_t control_TaskBuffer[ 128 ];
osStaticThreadDef_t control_TaskControlBlock;
osThreadId PS2_TaskHandle;
uint32_t PS2_TaskBuffer[ 128 ];
osStaticThreadDef_t PS2_TaskControlBlock;
osThreadId Data_TaskHandle;
uint32_t Data_TaskBuffer[ 128 ];
osStaticThreadDef_t Data_TaskControlBlock;
osThreadId MPU6050_TaskHandle;
uint32_t MPU6050_TaskBuffer[ 128 ];
osStaticThreadDef_t MPU6050_TaskControlBlock;
osThreadId Show_TaskHandle;
uint32_t Show_TaskBuffer[ 128 ];
osStaticThreadDef_t Show_TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void const * argument);
void StartPS2Task(void const * argument);
void StartDataTask(void const * argument);
void StartMPU6050Task(void const * argument);
void ShowTask(void const * argument);

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
  /* definition and creation of control_Task */
  osThreadStaticDef(control_Task, StartControlTask, osPriorityNormal, 0, 128, control_TaskBuffer, &control_TaskControlBlock);
  control_TaskHandle = osThreadCreate(osThread(control_Task), NULL);

  /* definition and creation of PS2_Task */
  osThreadStaticDef(PS2_Task, StartPS2Task, osPriorityNormal, 0, 128, PS2_TaskBuffer, &PS2_TaskControlBlock);
  PS2_TaskHandle = osThreadCreate(osThread(PS2_Task), NULL);

  /* definition and creation of Data_Task */
  osThreadStaticDef(Data_Task, StartDataTask, osPriorityNormal, 0, 128, Data_TaskBuffer, &Data_TaskControlBlock);
  Data_TaskHandle = osThreadCreate(osThread(Data_Task), NULL);

  /* definition and creation of MPU6050_Task */
  osThreadStaticDef(MPU6050_Task, StartMPU6050Task, osPriorityBelowNormal, 0, 128, MPU6050_TaskBuffer, &MPU6050_TaskControlBlock);
  MPU6050_TaskHandle = osThreadCreate(osThread(MPU6050_Task), NULL);

  /* definition and creation of Show_Task */
  osThreadStaticDef(Show_Task, ShowTask, osPriorityBelowNormal, 0, 128, Show_TaskBuffer, &Show_TaskControlBlock);
  Show_TaskHandle = osThreadCreate(osThread(Show_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief  Function implementing the control_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
	
	// ControlTaskTASK：小车控制周期任务
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms / 100Hz
	
  while(1)
  {
		osDelayUntil(&lastWakeTime,TimeIncrement);
		Get_Velocity(); // 获取小车当前速度
//		FourWheel_car_Motion_Inverse(Recive_Data.Sensor_str.X_speed,Recive_Data.Sensor_str.Z_speed); // 获取树莓派下发的速度值
		FourWheel_car_Motion_Inverse(ps2.X_speed,ps2.Z_speed);
		Motor_Control(PID_Incremental(&Moto1),PID_Incremental(&Moto2),PID_Incremental(&Moto3),PID_Incremental(&Moto4));

		
		

		
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartPS2Task */
/**
* @brief Function implementing the PS2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPS2Task */
void StartPS2Task(void const * argument)
{
  /* USER CODE BEGIN StartPS2Task */
  /* Infinite loop */
	
	//PS2Task：PS2遥控器任务
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms / 100Hz
	
  while(1)
  {
		osDelayUntil(&lastWakeTime,TimeIncrement);
		
		PS2_key = PS2_Receive();
		PS2_Pub_vel(PS2_key);
  }
  /* USER CODE END StartPS2Task */
}

/* USER CODE BEGIN Header_StartDataTask */
/**
* @brief Function implementing the Data_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataTask */
void StartDataTask(void const * argument)
{
  /* USER CODE BEGIN StartDataTask */
  /* Infinite loop */
	
	// DataTask ：发送数据给ROS任务
	
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(50); // 50ms / 20HZ
	
  while(1)
  {
    osDelayUntil(&lastWakeTime,TimeIncrement);
		
		STM32_TO_ROS(); // 通过串口把数据发送给ROS
		
  }
  /* USER CODE END StartDataTask */
}

/* USER CODE BEGIN Header_StartMPU6050Task */
/**
* @brief Function implementing the MPU6050_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPU6050Task */
void StartMPU6050Task(void const * argument)
{
  /* USER CODE BEGIN StartMPU6050Task */
  /* Infinite loop */
	
	// MPU6050Task：采集MPU6050数据
	
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms / 100HZ
  while(1)
  {
		osDelayUntil(&lastWakeTime,TimeIncrement);
		
    MPU_Get_Accelerometer(&acc); // 获取加速度值
		MPU_Get_Gyroscope(&gyro); // 获取角速度值
  }
  /* USER CODE END StartMPU6050Task */
}

/* USER CODE BEGIN Header_ShowTask */
/**
* @brief Function implementing the Show_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShowTask */
void ShowTask(void const * argument)
{
  /* USER CODE BEGIN ShowTask */
  /* Infinite loop */
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(100); // 100ms / 10HZ
  while(1)
  {
    osDelayUntil(&lastWakeTime,TimeIncrement);

//		OLED_ShowString(8,0,"Q-Robot",16,1);
//		OLED_Refresh();
  }
  /* USER CODE END ShowTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

