/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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
#define TTL_HNDLR huart3
#define IMU_I2C_HNDLR hi2c1
#define MD_IN_B_Pin GPIO_PIN_4
#define MD_IN_B_GPIO_Port GPIOA
#define MD_ISEN_B_Pin GPIO_PIN_5
#define MD_ISEN_B_GPIO_Port GPIOA
#define MD_ISEN_A_Pin GPIO_PIN_7
#define MD_ISEN_A_GPIO_Port GPIOA
#define MD_PWM_Pin GPIO_PIN_0
#define MD_PWM_GPIO_Port GPIOB
#define MCU_LED_Pin GPIO_PIN_12
#define MCU_LED_GPIO_Port GPIOB
#define SENSOR_EN_Pin GPIO_PIN_13
#define SENSOR_EN_GPIO_Port GPIOB
#define MD_IN_A_Pin GPIO_PIN_14
#define MD_IN_A_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA
#define INT2_XL_Pin GPIO_PIN_5
#define INT2_XL_GPIO_Port GPIOB
#define INT2_XL_EXTI_IRQn EXTI9_5_IRQn
#define INT_GYRO_Pin GPIO_PIN_8
#define INT_GYRO_GPIO_Port GPIOB
#define INT_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define INT_ACC_Pin GPIO_PIN_9
#define INT_ACC_GPIO_Port GPIOB
#define INT_ACC_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
