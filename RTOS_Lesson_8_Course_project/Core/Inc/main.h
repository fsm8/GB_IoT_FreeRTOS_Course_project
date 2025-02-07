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
#include "stm32g4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define deb5_Pin GPIO_PIN_4
#define deb5_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define deb2_Pin GPIO_PIN_6
#define deb2_GPIO_Port GPIOA
#define led4_Pin GPIO_PIN_7
#define led4_GPIO_Port GPIOA
#define deb4_Pin GPIO_PIN_0
#define deb4_GPIO_Port GPIOB
#define but3_Pin GPIO_PIN_10
#define but3_GPIO_Port GPIOB
#define deb7_Pin GPIO_PIN_6
#define deb7_GPIO_Port GPIOC
#define led2_Pin GPIO_PIN_7
#define led2_GPIO_Port GPIOC
#define deb6_Pin GPIO_PIN_8
#define deb6_GPIO_Port GPIOC
#define but4_Pin GPIO_PIN_8
#define but4_GPIO_Port GPIOA
#define led1_Pin GPIO_PIN_9
#define led1_GPIO_Port GPIOA
#define deb1_Pin GPIO_PIN_10
#define deb1_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define but2_Pin GPIO_PIN_4
#define but2_GPIO_Port GPIOB
#define but1_Pin GPIO_PIN_5
#define but1_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_6
#define led3_GPIO_Port GPIOB
#define deb3_Pin GPIO_PIN_9
#define deb3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

//#define TASK_DEBUG

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
