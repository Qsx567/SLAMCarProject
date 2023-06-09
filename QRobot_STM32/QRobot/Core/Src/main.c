/* USER CODE BEGIN Header */
#include "motor.h"
#include "encoder.h"
#include "system.h"
#include "motion_model.h"
#include "pid.h"
#include "delay.h"
#include "ps2.h"
#include "serial.h"
#include "mpu6050.h"

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
int PS2_key=0;
extern uint8_t uart5_rxbuff;
//extern int PS2_LX, PS2_LY, PS2_RX, PS2_RY;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t enc_cnt = 0;
	static uint16_t ps2_cnt = 0;
	static uint16_t uart_cnt = 0;
	static uint16_t imu_cnt = 0;
	
	if(htim == &htim6){
		enc_cnt ++;
		ps2_cnt ++;
		uart_cnt ++;
		imu_cnt ++;
		
		if(imu_cnt > 5){ // 5ms采集mpu6050数据
			imu_cnt = 0;
			MPU_Get_Accelerometer(&Send_Data.Sensor_str.Accelerometer);
			MPU_Get_Gyroscope(&Send_Data.Sensor_str.Gyroscope);
		}
		
		if(enc_cnt > 10){ // 10ms/100hz采集编码器数据
			enc_cnt = 0;
			Get_Velocity();
		}
		if(uart_cnt > 50){
			uart_cnt = 0;
			STM32_TO_ROS();
		}
			
		if(ps2_cnt > 60){
			ps2_cnt = 0;
			PS2_key = PS2_Receive();
			PS2_Pub_vel(PS2_key);
		}
		
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay_us_init(72);
	
	PS2_Init(); //PS2手柄初始化
	MPU6050_Init(); // MPU6050初始化
	
	// 电机PWM，满占空比是7200
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // M1_PWMA_1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // M1_PWMA_2
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // M3_PWMA_1
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // M3_PWMA_2
	
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // M4_PWMA_1
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2); // M4_PWMA_2
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); // M2_PWMA_1
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4); // M2_PWMA_2
	
	// 编码器
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL); // M1_ENC
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); // M2_ENC
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); // M3_ENC
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL); // M4_ENC
	
	Moto1.Target_Speed = 0.5;
	Moto2.Target_Speed = 0.5;
	Moto3.Target_Speed = 0.5;
	Moto4.Target_Speed = 0.5;
	
	// 定时器中断
	HAL_TIM_Base_Start_IT(&htim6); // 开启定时中断TIM6
	
	// 串口
	HAL_UART_Receive_IT(&huart5,(void *)&uart5_rxbuff,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_Delay(40);
//		Motor_Control(PID_Incremental(&Moto1),PID_Incremental(&Moto2),PID_Incremental(&Moto3),PID_Incremental(&Moto4));
//		

		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
