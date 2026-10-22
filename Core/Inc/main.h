/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define KEY0_Pin GPIO_PIN_2
#define KEY0_GPIO_Port GPIOE
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0E4_Pin GPIO_PIN_4
#define KEY0E4_GPIO_Port GPIOE
#define KEY0E4_EXTI_IRQn EXTI4_IRQn
#define TOUCH_CS_Pin GPIO_PIN_13
#define TOUCH_CS_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_8
#define LED0_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOF
#define LED2_Pin GPIO_PIN_10
#define LED2_GPIO_Port GPIOF
#define GAIN_A_Pin GPIO_PIN_4
#define GAIN_A_GPIO_Port GPIOA
#define GAIN_B_Pin GPIO_PIN_5
#define GAIN_B_GPIO_Port GPIOA
#define GAIN_C_Pin GPIO_PIN_6
#define GAIN_C_GPIO_Port GPIOA
#define TOUCH_SCK_Pin GPIO_PIN_0
#define TOUCH_SCK_GPIO_Port GPIOB
#define TOUCH_PEN_Pin GPIO_PIN_1
#define TOUCH_PEN_GPIO_Port GPIOB
#define TOUCH_MISO_Pin GPIO_PIN_2
#define TOUCH_MISO_GPIO_Port GPIOB
#define TOUCH_MOSI_Pin GPIO_PIN_11
#define TOUCH_MOSI_GPIO_Port GPIOF
#define GAIN2_A_Pin GPIO_PIN_12
#define GAIN2_A_GPIO_Port GPIOB
#define GAIN2_B_Pin GPIO_PIN_13
#define GAIN2_B_GPIO_Port GPIOB
#define GAIN2_C_Pin GPIO_PIN_14
#define GAIN2_C_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_15
#define LCD_BL_GPIO_Port GPIOB
#define AT24CXX_SCL_Pin GPIO_PIN_8
#define AT24CXX_SCL_GPIO_Port GPIOB
#define AT24CXX_SDA_Pin GPIO_PIN_9
#define AT24CXX_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
