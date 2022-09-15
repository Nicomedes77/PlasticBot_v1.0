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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct PETfilConv{
	//variables de estado
	bool 	soundAlarm_state;
	bool 	lightAlarm_state;
	bool 	col_state;
	//variables control de temp
	uint32_t currentExtTemp;
	uint32_t previousSetExtTemp;
	uint32_t setExtTemp;
	bool flagTemp_state;
}PETfilConv;

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
#define RotaryDT_Pin GPIO_PIN_13
#define RotaryDT_GPIO_Port GPIOC
#define RotaryDT_EXTI_IRQn EXTI15_10_IRQn
#define RotaryCLK_Pin GPIO_PIN_14
#define RotaryCLK_GPIO_Port GPIOC
#define RotaryCLK_EXTI_IRQn EXTI15_10_IRQn
#define Button_Pin GPIO_PIN_15
#define Button_GPIO_Port GPIOC
#define Button_EXTI_IRQn EXTI15_10_IRQn
#define RESET_MOTOR1_Pin GPIO_PIN_0
#define RESET_MOTOR1_GPIO_Port GPIOA
#define PASOS_MOTOR1_Pin GPIO_PIN_2
#define PASOS_MOTOR1_GPIO_Port GPIOA
#define CALENTADOR1_Pin GPIO_PIN_5
#define CALENTADOR1_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define Led_Pin GPIO_PIN_15
#define Led_GPIO_Port GPIOA
#define Detector_fil_Pin GPIO_PIN_3
#define Detector_fil_GPIO_Port GPIOB
#define Detector_fil_EXTI_IRQn EXTI3_IRQn
/* USER CODE BEGIN Private defines */

//Motor colector constants
#define APAGADO		0
#define VELOCIDAD_1	1
#define VELOCIDAD_2 2
#define VELOCIDAD_3	3

//Rotative Encoder constants
#define NoPressed	0
#define Ok			1
#define Left		2
#define Right		3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
