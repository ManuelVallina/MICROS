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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_ROJO_Pin GPIO_PIN_13
#define LED_ROJO_GPIO_Port GPIOC
#define INICIO_Pin GPIO_PIN_15
#define INICIO_GPIO_Port GPIOC
#define ECHO2_Pin GPIO_PIN_1
#define ECHO2_GPIO_Port GPIOA
#define TRIG_2_Pin GPIO_PIN_2
#define TRIG_2_GPIO_Port GPIOA
#define MOTOR2_Pin GPIO_PIN_7
#define MOTOR2_GPIO_Port GPIOA
#define MODO_1_Pin GPIO_PIN_2
#define MODO_1_GPIO_Port GPIOB
#define MODO_1_EXTI_IRQn EXTI2_IRQn
#define MODO_2_Pin GPIO_PIN_7
#define MODO_2_GPIO_Port GPIOE
#define MODO_2_EXTI_IRQn EXTI9_5_IRQn
#define TRIG_1_Pin GPIO_PIN_10
#define TRIG_1_GPIO_Port GPIOD
#define ECHO1_Pin GPIO_PIN_13
#define ECHO1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
