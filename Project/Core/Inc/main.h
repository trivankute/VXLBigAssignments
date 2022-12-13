/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
void buzzer_boi(uint8_t buzzer);
void send_uart(int value1,int value2);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PES_BUT_Pin GPIO_PIN_0
#define PES_BUT_GPIO_Port GPIOA
#define BUT_1_Pin GPIO_PIN_1
#define BUT_1_GPIO_Port GPIOA
#define BUT_2_Pin GPIO_PIN_4
#define BUT_2_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOA
#define BUT_3_Pin GPIO_PIN_0
#define BUT_3_GPIO_Port GPIOB
#define PES_RED_Pin GPIO_PIN_10
#define PES_RED_GPIO_Port GPIOB
#define PES_GREEN_Pin GPIO_PIN_8
#define PES_GREEN_GPIO_Port GPIOA
#define Traffic_1_RED_Pin GPIO_PIN_10
#define Traffic_1_RED_GPIO_Port GPIOA
#define Traffic_1_GREEN_Pin GPIO_PIN_3
#define Traffic_1_GREEN_GPIO_Port GPIOB
#define Traffic_2_GREEN_Pin GPIO_PIN_4
#define Traffic_2_GREEN_GPIO_Port GPIOB
#define Traffic_2_RED_Pin GPIO_PIN_5
#define Traffic_2_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
