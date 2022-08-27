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
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define CALENTADOR1_Pin GPIO_PIN_11
#define CALENTADOR1_GPIO_Port GPIOB
#define Led_Pin GPIO_PIN_15
#define Led_GPIO_Port GPIOA
#define Detector_fil_Pin GPIO_PIN_3
#define Detector_fil_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

typedef struct{

	float adc;
	float temp;

}adcTempTable;

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=3950 --max-adc=4095
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3950
// max adc: 4095

#define NUMTEMPS 25

static const adcTempTable tempNTC100K[NUMTEMPS] = {
		   {1, 1835},
		   {171, 286},
		   {341, 233},
		   {511, 206},
		   {681, 187},
		   {851, 173},
		   {1021, 161},
		   {1191, 151},
		   {1361, 142},
		   {1531, 135},
		   {1701, 127},
		   {1871, 121},
		   {2041, 114},
		   {2211, 108},
		   {2381, 102},
		   {2551, 96},
		   {2721, 90},
		   {2891, 83},
		   {3061, 77},
		   {3231, 70},
		   {3401, 62},
		   {3571, 53},
		   {3741, 41},
		   {3911, 25},
		   {4081, -23}
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
