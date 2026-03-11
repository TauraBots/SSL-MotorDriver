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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define H1B_Pin GPIO_PIN_0
#define H1B_GPIO_Port GPIOA
#define H1A_Pin GPIO_PIN_1
#define H1A_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_2
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_3
#define RX_GPIO_Port GPIOA
#define H2B_Pin GPIO_PIN_6
#define H2B_GPIO_Port GPIOA
#define H2A_Pin GPIO_PIN_7
#define H2A_GPIO_Port GPIOA
#define BATTERY_Pin GPIO_PIN_4
#define BATTERY_GPIO_Port GPIOC
#define M1IN1_Pin GPIO_PIN_0
#define M1IN1_GPIO_Port GPIOB
#define M1IN2_Pin GPIO_PIN_1
#define M1IN2_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define M2IN1_Pin GPIO_PIN_6
#define M2IN1_GPIO_Port GPIOC
#define M2IN2_Pin GPIO_PIN_7
#define M2IN2_GPIO_Port GPIOC
#define M4IN1_Pin GPIO_PIN_8
#define M4IN1_GPIO_Port GPIOC
#define M4IN2_Pin GPIO_PIN_9
#define M4IN2_GPIO_Port GPIOC
#define M3IN1_Pin GPIO_PIN_8
#define M3IN1_GPIO_Port GPIOA
#define M3IN2_Pin GPIO_PIN_11
#define M3IN2_GPIO_Port GPIOA
#define H3A_Pin GPIO_PIN_15
#define H3A_GPIO_Port GPIOA
#define H3B_Pin GPIO_PIN_3
#define H3B_GPIO_Port GPIOB
#define H4B_Pin GPIO_PIN_6
#define H4B_GPIO_Port GPIOB
#define H4A_Pin GPIO_PIN_7
#define H4A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
