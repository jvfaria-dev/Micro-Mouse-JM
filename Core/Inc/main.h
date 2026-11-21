/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SW1_1_Pin GPIO_PIN_14
#define SW1_1_GPIO_Port GPIOC
#define SW1_2_Pin GPIO_PIN_15
#define SW1_2_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_0
#define LED_2_GPIO_Port GPIOA
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOA
#define CargBat_Pin GPIO_PIN_2
#define CargBat_GPIO_Port GPIOA
#define SL1_Pin GPIO_PIN_3
#define SL1_GPIO_Port GPIOA
#define SL2_Pin GPIO_PIN_4
#define SL2_GPIO_Port GPIOA
#define SL3_Pin GPIO_PIN_5
#define SL3_GPIO_Port GPIOA
#define SL4_Pin GPIO_PIN_6
#define SL4_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_7
#define SW2_GPIO_Port GPIOA
#define SL5_Pin GPIO_PIN_1
#define SL5_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define AIN2_1_Pin GPIO_PIN_14
#define AIN2_1_GPIO_Port GPIOB
#define AIN1_1_Pin GPIO_PIN_15
#define AIN1_1_GPIO_Port GPIOB
#define PWM_1_Pin GPIO_PIN_8
#define PWM_1_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_9
#define PWM_2_GPIO_Port GPIOA
#define AIN2_2_Pin GPIO_PIN_10
#define AIN2_2_GPIO_Port GPIOA
#define AIN1_2_Pin GPIO_PIN_11
#define AIN1_2_GPIO_Port GPIOA
#define DigKey_Pin GPIO_PIN_9
#define DigKey_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
