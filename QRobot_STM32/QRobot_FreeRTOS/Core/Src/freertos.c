/* USER CODE BEGIN Header */
#include "motor.h"
#include "encoder.h"
#include "system.h"
#include "motion_model.h"
#include "pid.h"
#include "ps2.h"
#include "serial.h"
#include "mpu6050.h"


// ---------------- ���ļ�ΪFreeRTOS�������� -------------------------

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
osThreadId PS2_TaskHandle;
osThreadId Data_TaskHandle;
osThreadId MPU6050_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartControlTask(void const * argument);
void StartPS2Task(void const * argument);
void StartDataTask(void const * argument);
void StartMPU6050Task(void const * argument);

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
  osThreadDef(control_Task, StartControlTask, osPriorityNormal, 0, 128);
  control_TaskHandle = osThreadCreate(osThread(control_Task), NULL);

  /* definition and creation of PS2_Task */
  osThreadDef(PS2_Task, StartPS2Task, osPriorityNormal, 0, 128);
  PS2_TaskHandle = osThreadCreate(osThread(PS2_Task), NULL);

  /* definition and creation of Data_Task */
  osThreadDef(Data_Task, StartDataTask, osPriorityNormal, 0, 128);
  Data_TaskHandle = osThreadCreate(osThread(Data_Task), NULL);

  /* definition and creation of MPU6050_Task */
  osThreadDef(MPU6050_Task, StartMPU6050Task, osPriorityBelowNormal, 0, 128);
  MPU6050_TaskHandle = osThreadCreate(osThread(MPU6050_Task), NULL);

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
	
	// ControlTaskTASK��С��������������
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms / 100Hz
	
  while(1)
  {
		osDelayUntil(&lastWakeTime,TimeIncrement);
		Get_Velocity(); // ��ȡС����ǰ�ٶ�
//		FourWheel_car_Motion_Inverse(Recive_Data.Sensor_str.X_speed,Recive_Data.Sensor_str.Z_speed);
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
	
	//PS2Task��PS2ң��������
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
	
	// DataTask ���������ݸ�ROS����
	
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(50); // 50ms / 20HZ
	
  while(1)
  {
    osDelayUntil(&lastWakeTime,TimeIncrement);
		STM32_TO_ROS(); // ͨ�����ڰ����ݷ��͸�ROS
		
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
	
	// MPU6050Task���ɼ�MPU6050����
	
	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t TimeIncrement = pdMS_TO_TICKS(10); // 10ms / 100HZ
  while(1)
  {
		osDelayUntil(&lastWakeTime,TimeIncrement);
    MPU_Get_Accelerometer(&acc); // ��ȡ���ٶ�ֵ
		MPU_Get_Gyroscope(&gyro); // ��ȡ���ٶ�ֵ
  }
  /* USER CODE END StartMPU6050Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

