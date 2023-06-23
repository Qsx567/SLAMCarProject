/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOC
#define PS2_DAT_Pin GPIO_PIN_0
#define PS2_DAT_GPIO_Port GPIOC
#define PS2_COM_Pin GPIO_PIN_1
#define PS2_COM_GPIO_Port GPIOC
#define PS2_ATT_Pin GPIO_PIN_2
#define PS2_ATT_GPIO_Port GPIOC
#define PS2_CLK_Pin GPIO_PIN_3
#define PS2_CLK_GPIO_Port GPIOC
#define BEEP_Pin GPIO_PIN_7
#define BEEP_GPIO_Port GPIOA
#define MPU6050_SDA_Pin GPIO_PIN_4
#define MPU6050_SDA_GPIO_Port GPIOC
#define MPU6050_SCL_Pin GPIO_PIN_5
#define MPU6050_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_0
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_1
#define OLED_SCL_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
