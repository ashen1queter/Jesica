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
#include "stm32f4xx_hal.h"

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
#define DEBOUNCE_COUNT 5000
#define ENCODER_MODES 5
#define JOYSTICK_THROTTLE_Pin GPIO_PIN_0
#define JOYSTICK_THROTTLE_GPIO_Port GPIOA
#define JOYSTICK_ROLL_Pin GPIO_PIN_1
#define JOYSTICK_ROLL_GPIO_Port GPIOA
#define JOYSTICK_BTN_Pin GPIO_PIN_2
#define JOYSTICK_BTN_GPIO_Port GPIOA
#define WHEEL_BTN_Pin GPIO_PIN_3
#define WHEEL_BTN_GPIO_Port GPIOA
#define MULTI_BTN_Pin GPIO_PIN_4
#define MULTI_BTN_GPIO_Port GPIOA
#define PRG_BTN_Pin GPIO_PIN_5
#define PRG_BTN_GPIO_Port GPIOA
#define JOYSTICK_YAW_Pin GPIO_PIN_6
#define JOYSTICK_YAW_GPIO_Port GPIOA
#define JOYSTICK_PITCH_Pin GPIO_PIN_7
#define JOYSTICK_PITCH_GPIO_Port GPIOA
#define WHEEL_B_Pin GPIO_PIN_15
#define WHEEL_B_GPIO_Port GPIOA
#define WHEEL_A_Pin GPIO_PIN_3
#define WHEEL_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
