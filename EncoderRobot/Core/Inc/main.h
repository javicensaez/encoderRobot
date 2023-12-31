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
#include "stm32f3xx_hal.h"

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
#define IN5_Pin GPIO_PIN_0
#define IN5_GPIO_Port GPIOF
#define ENC2B_Pin GPIO_PIN_1
#define ENC2B_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define ENC2A_Pin GPIO_PIN_3
#define ENC2A_GPIO_Port GPIOA
#define ENC1A_Pin GPIO_PIN_4
#define ENC1A_GPIO_Port GPIOA
#define ENC1B_Pin GPIO_PIN_5
#define ENC1B_GPIO_Port GPIOA
#define ENC4B_Pin GPIO_PIN_6
#define ENC4B_GPIO_Port GPIOA
#define ENC4A_Pin GPIO_PIN_7
#define ENC4A_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_1
#define IN4_GPIO_Port GPIOB
#define IN6_Pin GPIO_PIN_8
#define IN6_GPIO_Port GPIOA
#define ENC3A_Pin GPIO_PIN_9
#define ENC3A_GPIO_Port GPIOA
#define ENC3B_Pin GPIO_PIN_10
#define ENC3B_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define IN8_Pin GPIO_PIN_4
#define IN8_GPIO_Port GPIOB
#define IN7_Pin GPIO_PIN_5
#define IN7_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_7
#define IN2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
