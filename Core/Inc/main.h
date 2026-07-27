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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ENCODR1_B_Pin GPIO_PIN_0
#define ENCODR1_B_GPIO_Port GPIOA
#define ENCODR1_A_Pin GPIO_PIN_1
#define ENCODR1_A_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_2
#define TX_GPIO_Port GPIOA
#define RX_Pin GPIO_PIN_3
#define RX_GPIO_Port GPIOA
#define ENCODER2_B_Pin GPIO_PIN_6
#define ENCODER2_B_GPIO_Port GPIOA
#define ENCODER2_A_Pin GPIO_PIN_7
#define ENCODER2_A_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_12
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_9
#define PWMB_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_3
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_4
#define OUT2_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_5
#define OUT3_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_6
#define OUT4_GPIO_Port GPIOB
#define OUT5_Pin GPIO_PIN_7
#define OUT5_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_8
#define SERVO_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
