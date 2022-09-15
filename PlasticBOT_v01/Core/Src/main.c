/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "gui.h"
#include "alarms.h"
#include "controlTemp.h"
#include "motors.h"

#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint32_t g_btnPressed = Ok;					//variable de control ENCODER ROTATIVO
uint32_t g_velocidad_Col1 = APAGADO;		//variable de control VELOCIDAD DE MOTOR - APAGADO = 0 mm/min | VELOCIDAD 1 = 300 mm/min | | VELOCIDAD 2 = 400 mm/min | VELOCIDAD 3 = 500 mm/min
bool g_alarmFil = false;					//variable de control de alarma por faltante de filamento
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
bool PetConv_Init(PETfilConv *_petFilConv);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	static uint32_t previousTick = 0;
	uint32_t currentTick = 0;
	static uint32_t currentScreen = pantallaWorkingScreen;
	static PETfilConv PETfilConv1;
	static uint32_t indexes[n_pantallas] = {0,0,0,0,0,0,0,0};

	/*
	indexes[0] -> NO SE UTILIZA
	indexes[1] -> Guarda Nº item seleccionado en menu principal
	indexes[2] -> Guarda Nº item seleccionado en submenu Extrusores
	indexes[3] -> Guarda Nº item seleccionado en submenu Colectores
	indexes[4] -> Guarda Nº item seleccionado en submenu Alarmas
	indexes[5] -> Guarda Nº item seleccionado en submenu Historial
	indexes[6] -> Guarda Nº item seleccionado en submenu "Acerca de..."
	indexes[7] -> Guarda Nº item seleccionado en submenu SelVelCol
	*/

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(!lcd_init())					Error_Handler();
  PresentacionLCD();
  if(!PetConv_Init(&PETfilConv1))	Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(g_btnPressed != NoPressed)
	  {
		  UpdateDataGUI(indexes,&g_btnPressed,&PETfilConv1,&currentScreen,&g_velocidad_Col1);
		  UpdateLCD(&PETfilConv1,&currentScreen);
		  UpdateCursor(indexes,&PETfilConv1,&currentScreen,&g_velocidad_Col1);
		  g_btnPressed = NoPressed;
	  }

	  ControlAlarms(&g_alarmFil,&PETfilConv1);
	  currentTick = HAL_GetTick();

	  if(currentTick - previousTick > 500)
	  {
		  Control_ON_OFF(&PETfilConv1);

		  if(currentScreen == pantallaWorkingScreen)
		  {
			  UpdateCurrentTemp(&PETfilConv1);
		  }

		  previousTick = currentTick;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 56000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin|PASOS_MOTOR1_Pin|CALENTADOR1_Pin|Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RotaryDT_Pin RotaryCLK_Pin Button_Pin */
  GPIO_InitStruct.Pin = RotaryDT_Pin|RotaryCLK_Pin|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_MOTOR1_Pin PASOS_MOTOR1_Pin CALENTADOR1_Pin Led_Pin */
  GPIO_InitStruct.Pin = RESET_MOTOR1_Pin|PASOS_MOTOR1_Pin|CALENTADOR1_Pin|Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Detector_fil_Pin */
  GPIO_InitStruct.Pin = Detector_fil_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Detector_fil_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */



bool PetConv_Init(PETfilConv *_petFilConv)
{
	_petFilConv -> lightAlarm_state = false;	//false = alarma luminica DESHABILITADA | true = alarma luminica HABILITADA
	_petFilConv -> soundAlarm_state = false;	//false = alarma sonora DESHABILITADA | true = alarma sonora HABILITADA
	_petFilConv -> col_state = false;			//false = motor del colector DESHABILITADO | true = motor del colector HABILITADO
	_petFilConv -> currentExtTemp = 0;
	_petFilConv -> previousSetExtTemp = 50;
	_petFilConv -> setExtTemp = 50;
	_petFilConv -> flagTemp_state = false;

	DesactivaCalentador();	//deshabilito CALENTADOR 1
	Colector_Init();		//inicio timer asociado a la velocidad del colector

	return true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t previousTick;
	uint32_t currentTick = HAL_GetTick();
	//Rotate encoder CCKW or CKW
	if(currentTick - previousTick > 300)
	{
		if(GPIO_Pin == RotaryDT_Pin)			g_btnPressed = Left;
		else if(GPIO_Pin == RotaryCLK_Pin)		g_btnPressed = Right;
		else if(GPIO_Pin == Button_Pin)			g_btnPressed = Ok;
		else if(GPIO_Pin == Detector_fil_Pin)
		{
			if(HAL_GPIO_ReadPin(GPIOB, Detector_fil_Pin))	g_alarmFil = false;
			else	g_alarmFil = true;
		}

		previousTick = currentTick;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t previousVel_Col1 = APAGADO;

	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 10-1 (VELOCIDAD 1: 100 mm/min)
	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 8-1 (VELOCIDAD 2: 200 mm/min)
	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 7-1 (VELOCIDAD 3: 300 mm/min)

	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 5-1 (VELOCIDAD 3: 500 mm/min)
	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 6-1 (VELOCIDAD 3: 400 mm/min)

	if(htim->Instance == TIM2)
	{
		if(g_velocidad_Col1 != previousVel_Col1)
		{
			switch(g_velocidad_Col1)
			{
				case VELOCIDAD_1:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 10-1);
					HAL_TIM_Base_Start_IT(htim);
					break;

				case VELOCIDAD_2:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 8-1);
					HAL_TIM_Base_Start_IT(htim);
					break;

				case VELOCIDAD_3:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 7-1);
					HAL_TIM_Base_Start_IT(htim);
					break;

				default: break;
			}

			previousVel_Col1 = g_velocidad_Col1;
		}

		HAL_GPIO_TogglePin(GPIOA, PASOS_MOTOR1_Pin);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  PrintErrorMessage();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
