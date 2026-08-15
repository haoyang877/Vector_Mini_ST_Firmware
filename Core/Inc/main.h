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
#define RGB_TIM_Pin GPIO_PIN_2
#define RGB_TIM_GPIO_Port GPIOA
#define ADC_TEMP_Pin GPIO_PIN_3
#define ADC_TEMP_GPIO_Port GPIOA
#define ADC_VBUS_Pin GPIO_PIN_4
#define ADC_VBUS_GPIO_Port GPIOA
#define ADC_IA_Pin GPIO_PIN_5
#define ADC_IA_GPIO_Port GPIOA
#define ADC_IB_Pin GPIO_PIN_6
#define ADC_IB_GPIO_Port GPIOA
#define PWM_AL_Pin GPIO_PIN_7
#define PWM_AL_GPIO_Port GPIOA
#define ADC_IC_Pin GPIO_PIN_4
#define ADC_IC_GPIO_Port GPIOC
#define PWM_BL_Pin GPIO_PIN_0
#define PWM_BL_GPIO_Port GPIOB
#define PWM_CL_Pin GPIO_PIN_1
#define PWM_CL_GPIO_Port GPIOB
#define BRD_ENC_CS_Pin GPIO_PIN_12
#define BRD_ENC_CS_GPIO_Port GPIOB
#define PWM_AH_Pin GPIO_PIN_8
#define PWM_AH_GPIO_Port GPIOA
#define PWM_BH_Pin GPIO_PIN_9
#define PWM_BH_GPIO_Port GPIOA
#define PWM_CH_Pin GPIO_PIN_10
#define PWM_CH_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_10
#define LED_G_GPIO_Port GPIOC
#define EXT_ENC_CS_Pin GPIO_PIN_11
#define EXT_ENC_CS_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
