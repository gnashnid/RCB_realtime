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
#define LED_STT_Pin GPIO_PIN_13
#define LED_STT_GPIO_Port GPIOC
#define LED_STT_ETH_Pin GPIO_PIN_14
#define LED_STT_ETH_GPIO_Port GPIOC
#define LED_BP_Pin GPIO_PIN_15
#define LED_BP_GPIO_Port GPIOC
#define BYPASS_Pin GPIO_PIN_2
#define BYPASS_GPIO_Port GPIOA
#define CS_Pin GPIO_PIN_4
#define CS_GPIO_Port GPIOA
#define CS_W25_Pin GPIO_PIN_12
#define CS_W25_GPIO_Port GPIOB
#define DE_Pin GPIO_PIN_8
#define DE_GPIO_Port GPIOA
#define S_OUT2_Pin GPIO_PIN_4
#define S_OUT2_GPIO_Port GPIOB
#define S_OUT1_Pin GPIO_PIN_5
#define S_OUT1_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_6
#define RST_GPIO_Port GPIOB
#define RST_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
