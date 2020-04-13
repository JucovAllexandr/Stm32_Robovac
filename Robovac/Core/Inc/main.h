/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define RM_2_1_Pin GPIO_PIN_2
#define RM_2_1_GPIO_Port GPIOE
#define RM_1_1_Pin GPIO_PIN_3
#define RM_1_1_GPIO_Port GPIOE
#define RM_EXTI4_ENC_Pin GPIO_PIN_4
#define RM_EXTI4_ENC_GPIO_Port GPIOE
#define RM_EXTI4_ENC_EXTI_IRQn EXTI4_IRQn
#define RM_SW_Pin GPIO_PIN_5
#define RM_SW_GPIO_Port GPIOE
#define MB_S2_1_Pin GPIO_PIN_14
#define MB_S2_1_GPIO_Port GPIOE
#define MB_S1_1_Pin GPIO_PIN_15
#define MB_S1_1_GPIO_Port GPIOE
#define MB_S1_2_Pin GPIO_PIN_10
#define MB_S1_2_GPIO_Port GPIOB
#define MB_S2_2_Pin GPIO_PIN_8
#define MB_S2_2_GPIO_Port GPIOC
#define RM_ENC_EN_Pin GPIO_PIN_5
#define RM_ENC_EN_GPIO_Port GPIOD
#define LM_SW_Pin GPIO_PIN_7
#define LM_SW_GPIO_Port GPIOD
#define RM_2_2PWM1_Pin GPIO_PIN_6
#define RM_2_2PWM1_GPIO_Port GPIOB
#define RM_1_2PWM2_Pin GPIO_PIN_7
#define RM_1_2PWM2_GPIO_Port GPIOB
#define LM_2_2_Pin GPIO_PIN_8
#define LM_2_2_GPIO_Port GPIOB
#define LM_1_2_Pin GPIO_PIN_9
#define LM_1_2_GPIO_Port GPIOB
#define LM_1_1_Pin GPIO_PIN_0
#define LM_1_1_GPIO_Port GPIOE
#define LM_2_1_Pin GPIO_PIN_1
#define LM_2_1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
