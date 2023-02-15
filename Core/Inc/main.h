/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define DEVICE_SCR		0x0
#define DEVICE_TRIAC	0x1

#define PHASE_SHIFT_ANGLE_TIM 6666
#define PI_ANGLE_TIM 10000
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
#define ANG_CNTR_A_Pin GPIO_PIN_0
#define ANG_CNTR_A_GPIO_Port GPIOA
#define ANG_CNTR_B_Pin GPIO_PIN_1
#define ANG_CNTR_B_GPIO_Port GPIOA
#define AUX_ADC1_Pin GPIO_PIN_2
#define AUX_ADC1_GPIO_Port GPIOA
#define AUX_ADC2_Pin GPIO_PIN_3
#define AUX_ADC2_GPIO_Port GPIOA
#define AUX_ADC3_Pin GPIO_PIN_4
#define AUX_ADC3_GPIO_Port GPIOA
#define AUX_ADC4_Pin GPIO_PIN_5
#define AUX_ADC4_GPIO_Port GPIOA
#define AUX_ADC5_Pin GPIO_PIN_6
#define AUX_ADC5_GPIO_Port GPIOA
#define AUX_ADC6_Pin GPIO_PIN_7
#define AUX_ADC6_GPIO_Port GPIOA
#define LED_USB_DETECT_Pin GPIO_PIN_13
#define LED_USB_DETECT_GPIO_Port GPIOB
#define FIRING_MODE_Pin GPIO_PIN_14
#define FIRING_MODE_GPIO_Port GPIOB
#define FIRING_MODE_EXTI_IRQn EXTI15_10_IRQn
#define DEVICE_MODE_Pin GPIO_PIN_15
#define DEVICE_MODE_GPIO_Port GPIOB
#define DEVICE_MODE_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
