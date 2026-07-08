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
#define LD5_Pin GPIO_PIN_0
#define LD5_GPIO_Port GPIOC
#define S5_Pin GPIO_PIN_1
#define S5_GPIO_Port GPIOC
#define S5_EXTI_IRQn EXTI1_IRQn
#define LD3_Pin GPIO_PIN_2
#define LD3_GPIO_Port GPIOC
#define LD4_Pin GPIO_PIN_3
#define LD4_GPIO_Port GPIOC
#define V08_Pin GPIO_PIN_0
#define V08_GPIO_Port GPIOA
#define IC2_Pin GPIO_PIN_1
#define IC2_GPIO_Port GPIOA
#define V50_Pin GPIO_PIN_4
#define V50_GPIO_Port GPIOC
#define IC3_Pin GPIO_PIN_5
#define IC3_GPIO_Port GPIOC
#define V09_Pin GPIO_PIN_0
#define V09_GPIO_Port GPIOB
#define V30_Pin GPIO_PIN_1
#define V30_GPIO_Port GPIOB
#define V20_Pin GPIO_PIN_2
#define V20_GPIO_Port GPIOB
#define V10_Pin GPIO_PIN_11
#define V10_GPIO_Port GPIOB
#define IC1_Pin GPIO_PIN_12
#define IC1_GPIO_Port GPIOB
#define V40_Pin GPIO_PIN_15
#define V40_GPIO_Port GPIOB
#define V60_Pin GPIO_PIN_8
#define V60_GPIO_Port GPIOA
#define V70_Pin GPIO_PIN_9
#define V70_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_15
#define LD1_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_10
#define S3_GPIO_Port GPIOC
#define S3_EXTI_IRQn EXTI15_10_IRQn
#define S2_Pin GPIO_PIN_11
#define S2_GPIO_Port GPIOC
#define S2_EXTI_IRQn EXTI15_10_IRQn
#define S4_Pin GPIO_PIN_12
#define S4_GPIO_Port GPIOC
#define S4_EXTI_IRQn EXTI15_10_IRQn
#define S1_Pin GPIO_PIN_2
#define S1_GPIO_Port GPIOD
#define S1_EXTI_IRQn EXTI2_IRQn
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
