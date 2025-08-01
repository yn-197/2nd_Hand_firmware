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
#include "stm32f7xx_hal.h"

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
#define SPI4_SS2_Pin GPIO_PIN_3
#define SPI4_SS2_GPIO_Port GPIOE
#define SPI4_SS1_Pin GPIO_PIN_4
#define SPI4_SS1_GPIO_Port GPIOE
#define SPI5_SS3_Pin GPIO_PIN_5
#define SPI5_SS3_GPIO_Port GPIOF
#define SPI5_SS2_Pin GPIO_PIN_6
#define SPI5_SS2_GPIO_Port GPIOF
#define SPI5_SS1_Pin GPIO_PIN_10
#define SPI5_SS1_GPIO_Port GPIOF
#define SPI2_SS2_Pin GPIO_PIN_0
#define SPI2_SS2_GPIO_Port GPIOC
#define SPI2_SS1_Pin GPIO_PIN_3
#define SPI2_SS1_GPIO_Port GPIOC
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOG
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOG
#define Buzzer_Pin GPIO_PIN_8
#define Buzzer_GPIO_Port GPIOG
#define SPI3_SS1_Pin GPIO_PIN_12
#define SPI3_SS1_GPIO_Port GPIOC
#define SPI3_SS2_Pin GPIO_PIN_0
#define SPI3_SS2_GPIO_Port GPIOD
#define SPI1_SS1_Pin GPIO_PIN_10
#define SPI1_SS1_GPIO_Port GPIOG
#define SPI1_SS2_Pin GPIO_PIN_12
#define SPI1_SS2_GPIO_Port GPIOG
#define SPI6_SS1_Pin GPIO_PIN_15
#define SPI6_SS1_GPIO_Port GPIOG
#define SPI6_SS2_Pin GPIO_PIN_3
#define SPI6_SS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
