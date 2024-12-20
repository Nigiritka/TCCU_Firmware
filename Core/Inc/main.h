/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>

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


void intToStr(int N, char *str);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAN_1_EN_Pin GPIO_PIN_0
#define FAN_1_EN_GPIO_Port GPIOF
#define FAN_2_EN_Pin GPIO_PIN_1
#define FAN_2_EN_GPIO_Port GPIOF
#define IOEXPANDER_INT_L_Pin GPIO_PIN_0
#define IOEXPANDER_INT_L_GPIO_Port GPIOA
#define IOEXPANDER_INT_L_EXTI_IRQn EXTI0_1_IRQn
#define FAN_3_EN_Pin GPIO_PIN_1
#define FAN_3_EN_GPIO_Port GPIOA
#define ADC_nCS_Pin GPIO_PIN_4
#define ADC_nCS_GPIO_Port GPIOA
#define ADC_IRQ_Pin GPIO_PIN_1
#define ADC_IRQ_GPIO_Port GPIOB
#define ADC_IRQ_EXTI_IRQn EXTI0_1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
