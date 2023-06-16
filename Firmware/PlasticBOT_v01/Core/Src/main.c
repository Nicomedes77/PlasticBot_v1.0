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
#include "i2c-lcd.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//enum Button {Ok, Left, Right, NoPressed};

//typedef enum{
//	APAGADO = 0,
//	VELOCIDAD_1 = 1,
//	VELOCIDAD_2 = 2,
//	VELOCIDAD_3 = 3,
//}vel_Col;

typedef struct PETfilConv{
	//variables de estado
	//bool 	filDetector_state;
	bool 	soundAlarm_state;
	bool 	lightAlarm_state;
	bool 	col_state;
	//variables de temp
	uint32_t currentExtTemp;
	uint32_t previousSetExtTemp;
	uint32_t setExtTemp;
	//variables de control PID
	float_t PID_p;
	float_t PID_i;
	float_t PID_d;
	float_t PID_error;
	float_t previous_error;
	float_t elapsedTime, time, timePrev;
	uint32_t factor_cor;
}PETfilConv;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	CLEAR_LCD		0x01
#define LINE1_LCD 		0x80|0x00
#define LINE2_LCD		0x80|0x40
#define LINE3_LCD		0x80|0x14
#define LINE4_LCD		0x80|0x54
#define C_LINE1_LCD		0x80|0x13	// direccion para imprimir cursor en linea 1
#define XC_LINE1_LCD	0x80|0x11	// direccion para imprimir X en linea 1
#define C_LINE2_LCD		0x80|0x53	// direccion para imprimir cursor en linea 2
#define XC_LINE2_LCD	0x80|0x51	// direccion para imprimir X en linea 2
#define C_LINE3_LCD		0x80|0x27	// direccion para imprimir cursor en linea 3
#define XC_LINE3_LCD	0x80|0x25	// direccion para imprimir X en linea 3
#define C_LINE4_LCD		0x80|0x67	// direccion para imprimir cursor en linea 4
#define XC_LINE4_LCD	0x80|0x65	// direccion para imprimir X en linea 4
#define LINES			4

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
/*
pantalla 0 - pantalla de trabajo
pantalla 11 - menu principal (parte 1)
pantalla 12 - menu principal (parte 2)
pantalla 2 - sub menu EXTRUSORES
pantalla 3 - sub menu COLECTORES fil.
pantalla 31 - sub menu selector velocidad colector fil.
pantalla 4 - sub menu ALARMAS
pantalla 5 - sub menu HISTORIAL
pantalla 6 - sub menu ACERCA DE...(parte 1)
pantalla 7 - sub menu ACERCA DE...(parte 2)
pantalla 8 - sub menu Seteo temp ExtrusorX
*/
#define pantallaWorkingScreen 				0
#define pantallaMainMenu_part1 				11
#define pantallaMainMenu_part2 				12
#define pantallaSubMenuExtrusores 			2
#define pantallaSubMenuColectoresFil 		3
#define pantallaSubMenuSelVelocidadColFil 	31
#define pantallaSubMenuAlarmas				4
#define pantallaSubMenuHistorial			5
#define pantallaSubMenuAcercaDe_part1		6
#define pantallaSubMenuAcercaDe_part2		7
#define pantallaSubmenuSetTempExt			8

#define cont_item_PrinMenu				1
#define cont_item_SubMenuExtrusores		2
#define cont_item_SubMenuColectores		3
#define cont_item_SubMenuAlarmas		4
#define cont_item_SubMenuHistorial		5
#define cont_item_SubMenuAcercaDe		6
#define cont_item_SubMenuSelVelCol		7


#define cantItemsPrinMenu 				6
#define cantItemsSubMenuExtrusores 		2
#define cantItemsSubMenuColectores 		2
#define cantItemsSubMenuAlarmas			3
#define cantItemsSubMenuHistorial		3
#define cantItemsSubMenuAcercaDe		2
#define cantItemsSubMenuSelVelCol		4
#define n_pantallas						8

//uint32_t previousTempExt = 0;
//uint32_t previousSetTempExt = 0;	//static dentro de updateDataGUI?
// uint32_t previousTick = 0;			//static dentro de ControlPID?
// uint32_t currentTick = 0;

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

//+++++++++++++++ variables, constantes y defines de control PID +++++++++++++

#define termistorNominalRes 100000
#define termistorNominalTemp 25
#define termistorBValue 3950
#define voltageDividerResistor 100000
#define N_SAMPLES_ADC 16
#define	CLEAR_LCD	0x01

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void PetConv_Init(PETfilConv *_petFilConv);
void PresentacionLCD(void);
void updateDataGUI(uint32_t *_indexes , uint32_t *_btn , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col);
void updateLCD(PETfilConv *_petFilConv , uint32_t *currentScreen);
void printScreenSettingTemp(PETfilConv *_petFilConv);
void printSettingTemp(PETfilConv *_petFilConv);
void printscreenWorking(PETfilConv *_petFilConv);
void printCurrentTemp(PETfilConv *_petFilConv);
void updateCursor(uint32_t *_indexes , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col);
void printX(uint8_t i);
void printCursor(uint8_t index);
void clearAllCursor(void);
void printScreen(char *screen[]);
void clearScreen(void);
void ControlMotor(PETfilConv *_petFilConv);
float_t getTemp(void);
//void controlPID(PETfilConv *_petFilConv);
void Control_ON_OFF(PETfilConv *_petFilConv);
void ControlAlarms(bool *_sensorFil , PETfilConv *_petFilConv);
void ActivaAlarmaLuminica(void);
void ActivaAlarmaSonora(void);
void DesactivaAlarmas(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//+++++++++++ variables usadas en control PID ++++++++++++

//variables and/temp
uint32_t bufferDMA;		//static dentro de getTemp()?
uint32_t tempConv;		//static dentro de getTemp()?
uint32_t i = 0;
float_t Vsupply = 3.3;
float_t Vout;
float_t R_NTC;
float_t B_param = 3950;
float_t T0 = 298.15;
float_t Temp_K;
//float_t Temp_C;
uint32_t tempCint;

//variables control PID
//uint32_t PWM_pin;	//esta variable seria el pin por donde sale el PWM controlado
//float_t temperature_read; //esta variable vendria a ser la que guarda el resultado de getTemp()
//float_t set_temperature = 100;
//float_t PID_error = 0;
//float_t previous_error = 0;
//float_t elapsedTime, Time, timePrev;
//uint32_t PID_value = 0;
//uint32_t factor_cor = 20;

//PID constants
#define kp  9.1
#define ki  1.5		//0.3 - 0.8 - 1.5
#define kd  2.3		//1.8 - 2.3

//Rotative Encoder constants
#define NoPressed	0
#define Ok			1
#define Left		2
#define Right		3

//Motor colector constants
#define APAGADO		0
#define VELOCIDAD_1	1
#define VELOCIDAD_2 2
#define VELOCIDAD_3	3


uint32_t g_btnPressed = Ok;				//variable de control ENCODER ROTATIVO
uint32_t g_velocidad_Col1 = APAGADO;		//variable de control VELOCIDAD DE MOTOR
bool g_alarmFil = false;		//variable de control de alarma por faltante de filamento
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
  lcd_init();
  PresentacionLCD();
  PetConv_Init(&PETfilConv1);
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(g_btnPressed != NoPressed)
	  {
		  updateDataGUI(indexes,&g_btnPressed,&PETfilConv1,&currentScreen,&g_velocidad_Col1);
		  updateLCD(&PETfilConv1,&currentScreen);
		  updateCursor(indexes,&PETfilConv1,&currentScreen,&g_velocidad_Col1);
		  g_btnPressed = NoPressed;
	  }

	  //Control_ON_OFF(&PETfilConv1);
	  ControlAlarms(&g_alarmFil,&PETfilConv1);
	  currentTick = HAL_GetTick();
	  if((currentTick - previousTick > 1000) && (currentScreen == pantallaWorkingScreen))
	  {
		  printCurrentTemp(&PETfilConv1);
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
  HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin|PASOS_MOTOR1_Pin|Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Buzzer_Pin|CALENTADOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RotaryDT_Pin RotaryCLK_Pin Button_Pin */
  GPIO_InitStruct.Pin = RotaryDT_Pin|RotaryCLK_Pin|Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_MOTOR1_Pin PASOS_MOTOR1_Pin Led_Pin */
  GPIO_InitStruct.Pin = RESET_MOTOR1_Pin|PASOS_MOTOR1_Pin|Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin CALENTADOR1_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|CALENTADOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

void PresentacionLCD(void)
{
 char *screenStart[] = {
		"     PlasticBOT     ",
		"        v0.1        ",
		"                    ",
		"   CFP nro 8 SMATA  "
	};


	clearScreen();
	HAL_Delay(100);
	printScreen(screenStart);
	HAL_Delay(4000);
	clearScreen();
}

void PetConv_Init(PETfilConv *_petFilConv)
{
	//_petFilConv -> filDetector_state = false;	//false = detector de filamento DESHABILITADO | true = detector de filamento HABILITADO
	_petFilConv -> lightAlarm_state = false;	//false = alarma luminica DESHABILITADA | true = alarma luminica HABILITADA
	_petFilConv -> soundAlarm_state = false;	//false = alarma sonora DESHABILITADA | true = alarma sonora HABILITADA
	_petFilConv -> col_state = false;			//false = motor del colector DESHABILITADO | true = motor del colector HABILITADO
	//_petFilConv -> velocidad_Col = APAGADO;		// APAGADO = 0 mm/min | VELOCIDAD 1 = 300 mm/min | | VELOCIDAD 2 = 400 mm/min | VELOCIDAD 3 = 500 mm/min
	_petFilConv -> currentExtTemp = 0;
	_petFilConv -> previousSetExtTemp = 25;
	_petFilConv -> setExtTemp = 25;
	_petFilConv -> PID_d = 0;
	_petFilConv -> PID_error = 0;
	_petFilConv -> PID_i = 0;
	_petFilConv -> PID_p = 0;
	_petFilConv -> elapsedTime = 0;
	_petFilConv -> factor_cor = 20;
	_petFilConv -> previous_error = 0;
	_petFilConv -> time = HAL_GetTick();
	_petFilConv -> timePrev = 0;

	//deshabilito MOTOR COL 1
	HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_RESET);
	//deshabilito CALENTADOR 1
	HAL_GPIO_WritePin(GPIOB, CALENTADOR1_Pin, GPIO_PIN_SET);
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
			if(HAL_GPIO_ReadPin(GPIOB, Detector_fil_Pin))
			{
				g_alarmFil = false;
			}
			else
			{
				g_alarmFil = true;
			}
		}

		previousTick = currentTick;
	}
}

void ControlAlarms(bool *_sensFil , PETfilConv *_petFilConv)
{
	if(*_sensFil)
	{
		if(_petFilConv -> lightAlarm_state)
		{
			ActivaAlarmaLuminica();
		}

		if(_petFilConv -> soundAlarm_state)
		{
			ActivaAlarmaSonora();
		}
	}

	else
	{
		DesactivaAlarmas();
	}
}

void ActivaAlarmaLuminica(void)
{
	HAL_GPIO_WritePin(GPIOA, Led_Pin, GPIO_PIN_SET);
}

void ActivaAlarmaSonora(void)
{
	HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_SET);
}

void DesactivaAlarmas(void)
{
	HAL_GPIO_WritePin(GPIOA, Led_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, Buzzer_Pin, GPIO_PIN_RESET);
}


void updateDataGUI(uint32_t *_indexes , uint32_t *_btn , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col)
{
	if(*_btn != NoPressed)
	{
			if(*_btn == Ok)
			{
				switch(*currentScreen)
				{
					case pantallaWorkingScreen:	//pantalla de trabajo
						*currentScreen = pantallaMainMenu_part1;
						_indexes[cont_item_PrinMenu] = 0;
					break;

					case pantallaMainMenu_part1: //menu principal (parte 1)
						if(_indexes[cont_item_PrinMenu] == 0) *currentScreen = pantallaWorkingScreen;
						else if(_indexes[cont_item_PrinMenu] == 1) *currentScreen = pantallaSubMenuExtrusores;
						else if(_indexes[cont_item_PrinMenu] == 2) *currentScreen = pantallaSubMenuColectoresFil;
						else if(_indexes[cont_item_PrinMenu] == 3) *currentScreen = pantallaSubMenuAlarmas;
					break;

					case pantallaMainMenu_part2: //menu principal (parte 2)
						if(_indexes[cont_item_PrinMenu] == 4) *currentScreen = pantallaSubMenuHistorial;
						else if(_indexes[cont_item_PrinMenu] == 5) *currentScreen = pantallaSubMenuAcercaDe_part1;
					break;

					case pantallaSubMenuExtrusores: //sub EXTRUSORES
						if(_indexes[cont_item_SubMenuExtrusores] == 0) *currentScreen = pantallaMainMenu_part1;
						else if (_indexes[cont_item_SubMenuExtrusores] == 1) *currentScreen = pantallaSubmenuSetTempExt;
					break;

					case pantallaSubMenuColectoresFil:	//sub COLECTORES fil.
						if(_indexes[cont_item_SubMenuColectores] == 0) *currentScreen = pantallaMainMenu_part1;
						else if(_indexes[cont_item_SubMenuColectores] == 1)	*currentScreen = pantallaSubMenuSelVelocidadColFil;
					break;

					case pantallaSubMenuAlarmas:	//sub menu ALARMAS
						if(_indexes[cont_item_SubMenuAlarmas] == 0) *currentScreen = pantallaMainMenu_part1;
						else if(_indexes[cont_item_SubMenuAlarmas] == 1)
						{
							if(_petFilConv -> lightAlarm_state == false)	_petFilConv -> lightAlarm_state = true;
							else	_petFilConv -> lightAlarm_state = false;
							//changeAlarms = true;
						}
						else if(_indexes[cont_item_SubMenuAlarmas] == 2)
						{
							if(_petFilConv -> soundAlarm_state == false)	_petFilConv -> soundAlarm_state = true;
							else	_petFilConv -> soundAlarm_state = false;
							//changeAlarms = true;
						}

						//if((_petFilConv -> soundAlarm_state == true)||(_petFilConv -> lightAlarm_state == true))	_petFilConv -> filDetector_state = true;
						//else _petFilConv -> filDetector_state = false;
					break;

					case pantallaSubMenuHistorial:	//sub HISTORIAL
						if(_indexes[cont_item_SubMenuHistorial] == 0) *currentScreen = pantallaMainMenu_part2;
						else{} //borra el valor acumulador del extrusor
					break;

					case pantallaSubMenuSelVelocidadColFil:	//sub selector velocidad

						if(_indexes[cont_item_SubMenuSelVelCol] == 0)
						{
							if(*_vel_col != APAGADO)
							{
								*_vel_col = APAGADO;
								_petFilConv -> col_state = false;
								ControlMotor(_petFilConv);
							}
						}

						else if(_indexes[cont_item_SubMenuSelVelCol] == 1)
						{
							if(*_vel_col != VELOCIDAD_1)
							{
								*_vel_col = VELOCIDAD_1;
								_petFilConv -> col_state = true;
								ControlMotor(_petFilConv);
							}
						}

						else if(_indexes[cont_item_SubMenuSelVelCol] == 2)
						{
							if(*_vel_col != VELOCIDAD_2)
							{
								*_vel_col = VELOCIDAD_2;
								_petFilConv -> col_state = true;
								ControlMotor(_petFilConv);
							}
						}
						else if(_indexes[cont_item_SubMenuSelVelCol] == 3)
						{
							if(*_vel_col != VELOCIDAD_3)
							{
								*_vel_col = VELOCIDAD_3;
								_petFilConv -> col_state = true;
								ControlMotor(_petFilConv);
							}
						}

						*currentScreen = pantallaSubMenuColectoresFil;
					break;

					case pantallaSubMenuAcercaDe_part1:	//sub menu ACERCA DE...
						*currentScreen = pantallaMainMenu_part2;
					break;

					case pantallaSubMenuAcercaDe_part2:	//sub menu ACERCA DE...
						*currentScreen = pantallaMainMenu_part2;
					break;

					case pantallaSubmenuSetTempExt:	//menu seteo de temperatura COLECTORES
						_petFilConv -> setExtTemp = _petFilConv -> previousSetExtTemp;
						*currentScreen = pantallaSubMenuExtrusores;
					break;

					default: break;
				}
			}

			else if(*_btn == Left)
			{
					switch(*currentScreen)
					{
						case pantallaWorkingScreen:	//pantalla de trabajo
							//NADA
						break;

						case pantallaMainMenu_part1:
							if(_indexes[cont_item_PrinMenu] == 0) _indexes[cont_item_PrinMenu] = 0;
							else	_indexes[cont_item_PrinMenu]--;
						break;

						case pantallaMainMenu_part2:
							if(_indexes[cont_item_PrinMenu] <= 4)
							{
								*currentScreen = pantallaMainMenu_part1;
								_indexes[cont_item_PrinMenu] = 3;
							}
							else	_indexes[cont_item_PrinMenu]--;
						break;

						case pantallaSubMenuSelVelocidadColFil:
							if(_indexes[cont_item_SubMenuSelVelCol] == 0) _indexes[cont_item_SubMenuSelVelCol] = 0;
							else	_indexes[cont_item_SubMenuSelVelCol]--;
						break;

						case pantallaSubMenuAcercaDe_part2:	*currentScreen = pantallaSubMenuAcercaDe_part1;
						break;

						case pantallaSubmenuSetTempExt:


						//	if(_petFilConv -> setExtTemp == 0)	_petFilConv -> setExtTemp = 0;
						//	else _petFilConv -> setExtTemp--;
							if(_petFilConv -> previousSetExtTemp == 0) _petFilConv -> previousSetExtTemp = 0;
							else _petFilConv -> previousSetExtTemp--;
						break;

						default:
							if(_indexes[*currentScreen] == 0) _indexes[*currentScreen] = 0;
							else _indexes[*currentScreen]--;
					}
			}


			else if (*_btn == Right)
			{
					switch(*currentScreen)
					{
						case pantallaWorkingScreen:	//pantalla de trabajo
							//NADA
						break;

						case pantallaMainMenu_part1:
							if(_indexes[cont_item_PrinMenu] >= 3)
							{
								*currentScreen = pantallaMainMenu_part2;
								_indexes[cont_item_PrinMenu] = 4;
							}
							else	_indexes[cont_item_PrinMenu]++;
						break;

						case pantallaMainMenu_part2:
							if(_indexes[cont_item_PrinMenu] >= 5) _indexes[cont_item_PrinMenu] = 5;
							else	_indexes[cont_item_PrinMenu]++;
						break;

						case pantallaSubMenuExtrusores:
							if(_indexes[*currentScreen] == (cantItemsSubMenuExtrusores - 1)) _indexes[*currentScreen] = cantItemsSubMenuExtrusores - 1;
							else _indexes[*currentScreen]++;
						break;

						case pantallaSubMenuColectoresFil:
							if(_indexes[*currentScreen] == (cantItemsSubMenuColectores - 1)) _indexes[*currentScreen] = cantItemsSubMenuColectores - 1;
							else _indexes[*currentScreen]++;
						break;

						case pantallaSubMenuSelVelocidadColFil:
							if(_indexes[cont_item_SubMenuSelVelCol] >= 3) _indexes[cont_item_PrinMenu] = 3;
							else	_indexes[cont_item_SubMenuSelVelCol]++;
						break;

						case pantallaSubMenuAlarmas:
							if(_indexes[*currentScreen] == (cantItemsSubMenuAlarmas - 1)) _indexes[*currentScreen] = cantItemsSubMenuAlarmas - 1;
							else _indexes[*currentScreen]++;
						break;

						case pantallaSubMenuHistorial:
							if(_indexes[*currentScreen] == (cantItemsSubMenuHistorial - 1)) _indexes[*currentScreen] = cantItemsSubMenuHistorial - 1;
							else _indexes[*currentScreen]++;
						break;

						case pantallaSubMenuAcercaDe_part1: *currentScreen = pantallaSubMenuAcercaDe_part2;
						break;

						case pantallaSubmenuSetTempExt:	//menu seteo de temperatura COLECTORES

							//if(_petFilConv -> setExtTemp == 250) _petFilConv -> setExtTemp = 250;
							//else _petFilConv -> setExtTemp++;	//Temp++
							if(_petFilConv -> previousSetExtTemp == 250) _petFilConv -> previousSetExtTemp = 250;
							else _petFilConv -> previousSetExtTemp++;	//Temp++
						break;

						default: break;
					}
			}
	}
}

/*
void updateDataGUI(PETfilConv *_petFilConv)
{
	if(btnPressed != NoPressed)
	{
		switch(btnPressed)
		{
			case Ok:
				switch(currentScreen)
				{
					case pantallaWorkingScreen:	//pantalla de trabajo
						currentScreen = pantallaMainMenu_part1;
						indexes[cont_item_PrinMenu] = 0;
					break;

					case pantallaMainMenu_part1: //menu principal (parte 1)
						if(indexes[cont_item_PrinMenu] == 0) currentScreen = pantallaWorkingScreen;
						else if(indexes[cont_item_PrinMenu] == 1) currentScreen = pantallaSubMenuExtrusores;
						else if(indexes[cont_item_PrinMenu] == 2) currentScreen = pantallaSubMenuColectoresFil;
						else if(indexes[cont_item_PrinMenu] == 3) currentScreen = pantallaSubMenuAlarmas;
					break;

					case pantallaMainMenu_part2: //menu principal (parte 2)
						if(indexes[cont_item_PrinMenu] == 4) currentScreen = pantallaSubMenuHistorial;
						else if(indexes[cont_item_PrinMenu] == 5) currentScreen = pantallaSubMenuAcercaDe_part1;
					break;

					case pantallaSubMenuExtrusores: //sub EXTRUSORES
						if(indexes[cont_item_SubMenuExtrusores] == 0) currentScreen = pantallaMainMenu_part1;
						else if (indexes[cont_item_SubMenuExtrusores] == 1) currentScreen = pantallaSubmenuSetTempExt;
					break;

					case pantallaSubMenuColectoresFil:	//sub COLECTORES fil.
						if(indexes[cont_item_SubMenuColectores] == 0) currentScreen = pantallaMainMenu_part1;
						else if(indexes[cont_item_SubMenuColectores] == 1)	currentScreen = pantallaSubMenuSelVelocidadColFil;
					break;

					case pantallaSubMenuAlarmas:	//sub menu ALARMAS
						if(indexes[cont_item_SubMenuAlarmas] == 0) currentScreen = pantallaMainMenu_part1;
						else if(indexes[cont_item_SubMenuAlarmas] == 1)
						{
							if(_petFilConv -> lightAlarm_state == false)	_petFilConv -> lightAlarm_state = true;
							else	_petFilConv ->lightAlarm_state = false;
							changeAlarms = true;
						}
						else if(indexes[cont_item_SubMenuAlarmas] == 2)
						{
							if(_petFilConv -> soundAlarm_state == false)	_petFilConv -> soundAlarm_state = true;
							else	_petFilConv -> soundAlarm_state = false;
							changeAlarms = true;
						}

						if((_petFilConv -> soundAlarm_state == true)||(_petFilConv -> lightAlarm_state == true))	_petFilConv -> filDetector_state = true;
						else _petFilConv -> filDetector_state = false;
					break;

					case pantallaSubMenuHistorial:	//sub HISTORIAL
						if(indexes[cont_item_SubMenuHistorial] == 0) currentScreen = pantallaMainMenu_part2;
						else{} //borra el valor acumulador del extrusor
					break;

					case pantallaSubMenuSelVelocidadColFil:	//sub selector velocidad

						if(indexes[cont_item_SubMenuSelVelCol] == 0)
						{
							if(_petFilConv -> velocidad_Col != APAGADO)
							{
								_petFilConv -> velocidad_Col = APAGADO;
								_petFilConv -> col_state = false;
								ControlMotor();
							}
						}

						else if(indexes[cont_item_SubMenuSelVelCol] == 1)
						{
							if(_petFilConv -> velocidad_Col != VELOCIDAD_1)
							{
								_petFilConv -> velocidad_Col = VELOCIDAD_1;
								_petFilConv -> col_state = true;
								ControlMotor();
							}
						}

						else if(indexes[cont_item_SubMenuSelVelCol] == 2)
						{
							if(_petFilConv -> velocidad_Col != VELOCIDAD_2)
							{
								_petFilConv -> velocidad_Col = VELOCIDAD_2;
								_petFilConv -> col_state = true;
								ControlMotor();
							}
						}
						else if(indexes[cont_item_SubMenuSelVelCol] == 3)
						{
							if(_petFilConv -> velocidad_Col != VELOCIDAD_3)
							{
								_petFilConv -> velocidad_Col = VELOCIDAD_3;
								_petFilConv -> col_state = true;
								ControlMotor();
							}
						}

						currentScreen = pantallaSubMenuColectoresFil;
					break;

					case pantallaSubMenuAcercaDe_part1:	//sub menu ACERCA DE...
						currentScreen = pantallaMainMenu_part2;
					break;

					case pantallaSubMenuAcercaDe_part2:	//sub menu ACERCA DE...
						currentScreen = pantallaMainMenu_part2;
					break;

					case pantallaSubmenuSetTempExt:	//menu seteo de temperatura COLECTORES
						currentScreen = pantallaSubMenuExtrusores;
					break;
				}
				break;

				case Left:
					switch(currentScreen)
					{
						case pantallaWorkingScreen:	//pantalla de trabajo
							//NADA
						break;

						case pantallaMainMenu_part1:
							if(indexes[cont_item_PrinMenu] == 0) indexes[cont_item_PrinMenu] = 0;
							else	indexes[cont_item_PrinMenu]--;
						break;

						case pantallaMainMenu_part2:
							if(indexes[cont_item_PrinMenu] <= 4)
							{
								currentScreen = pantallaMainMenu_part1;
								indexes[cont_item_PrinMenu] = 3;
							}
							else	indexes[cont_item_PrinMenu]--;
						break;

						case pantallaSubMenuSelVelocidadColFil:
							if(indexes[cont_item_SubMenuSelVelCol] == 0) indexes[cont_item_SubMenuSelVelCol] = 0;
							else	indexes[cont_item_SubMenuSelVelCol]--;
						break;

						case pantallaSubMenuAcercaDe_part2:	currentScreen = pantallaSubMenuAcercaDe_part1;
						break;

						case pantallaSubmenuSetTempExt:
							if(_petFilConv -> setExtTemp == 0)	_petFilConv -> setExtTemp = 0;
							else _petFilConv -> setExtTemp--;
						break;

						default:
							if(indexes[currentScreen] == 0) indexes[currentScreen] = 0;
							else indexes[currentScreen]--;
					}
				break;

				case Right:
					switch(currentScreen)
					{
						case pantallaWorkingScreen:	//pantalla de trabajo
							//NADA
						break;

						case pantallaMainMenu_part1:
							if(indexes[cont_item_PrinMenu] >= 3)
							{
								currentScreen = pantallaMainMenu_part2;
								indexes[cont_item_PrinMenu] = 4;
							}
							else	indexes[cont_item_PrinMenu]++;
						break;

						case pantallaMainMenu_part2:
							if(indexes[cont_item_PrinMenu] >= 5) indexes[cont_item_PrinMenu] = 5;
							else	indexes[cont_item_PrinMenu]++;
						break;

						case pantallaSubMenuExtrusores:
							if(indexes[currentScreen] == (cantItemsSubMenuExtrusores - 1)) indexes[currentScreen] = cantItemsSubMenuExtrusores - 1;
							else indexes[currentScreen]++;
						break;

						case pantallaSubMenuColectoresFil:
							if(indexes[currentScreen] == (cantItemsSubMenuColectores - 1)) indexes[currentScreen] = cantItemsSubMenuColectores - 1;
							else indexes[currentScreen]++;
						break;

						case pantallaSubMenuSelVelocidadColFil:
							if(indexes[cont_item_SubMenuSelVelCol] >= 3) indexes[cont_item_PrinMenu] = 3;
							else	indexes[cont_item_SubMenuSelVelCol]++;
						break;

						case pantallaSubMenuAlarmas:
							if(indexes[currentScreen] == (cantItemsSubMenuAlarmas - 1)) indexes[currentScreen] = cantItemsSubMenuAlarmas - 1;
							else indexes[currentScreen]++;
						break;

						case pantallaSubMenuHistorial:
							if(indexes[currentScreen] == (cantItemsSubMenuHistorial - 1)) indexes[currentScreen] = cantItemsSubMenuHistorial - 1;
							else indexes[currentScreen]++;
						break;

						case pantallaSubMenuAcercaDe_part1: currentScreen = pantallaSubMenuAcercaDe_part2;
						break;

						case pantallaSubmenuSetTempExt:	//menu seteo de temperatura COLECTORES
							if(_petFilConv -> setExtTemp == 250) _petFilConv -> setExtTemp = 250;
							else _petFilConv -> setExtTemp++;	//Temp++
						break;
					}
				break;
		}

		btnPressed = NoPressed;
	}
}
*/

void updateLCD(PETfilConv *_petFilConv , uint32_t *currentScreen)
{
	static uint32_t previousScreen;

	char *prinMenu1[] = {
	    "Pantalla Principal ",
	    "Extrusores         ",
	    "Colectores fil.    ",
	    "Alarmas            "
	};

	char *prinMenu2[] = {
	    "Historial          ",
	    "Acerca de...       ",
	    "                   ",
		"                   "
	};

	char *subMenuExtrusores[] = {
	    "Volver...          ",
	    "Extrusor 1         ",
		"                   ",
		"                   "
	};

	char *subMenuColectores[] = {
	    "Volver...          ",
	    "Colector fil 1     ",
		"                   ",
		"                   "
	};

	char *subMenuAlarmas[] = {
	    "Volver...          ",
	    "Luminica           ",
	    "Sonora             ",
		"                   "
	};

	char *subMenuHistorial[] = {
	    "Volver...          ",
	    "Extrusor 1         ",
	    "Borrar historial   ",
		"                   "
	};

	char *subMenuAcercaDe1[] = {
	    " Proyecto final de  ",
	    "       Esp. en      ",
	    " Sistemas Embebidos ",
		"    LSE - FiUBA     "
	};

	char *subMenuAcercaDe2[] = {
	    "    Ing. Nicolas    ",
	    "    Vargas Alice    ",
		"                    ",
		"        2022        "
	};

	char *subMenuSelVelocidadColec[] = {
	    "Apagado            ",
	    "300 mm/min         ",
		"400 mm/min         ",
		"500 mm/min         "
	};

	if(*currentScreen != previousScreen) //si la pantalla actual es distinto a la anterior o si la pantalla actual es la de seteo de temp de extrusor
	{
		previousScreen = *currentScreen;

		switch(*currentScreen)
		{
			case pantallaWorkingScreen: printscreenWorking(_petFilConv);
			break;

			case pantallaMainMenu_part1: printScreen(prinMenu1);
			break;

			case pantallaMainMenu_part2: printScreen(prinMenu2);
			break;

			case pantallaSubMenuExtrusores: printScreen(subMenuExtrusores);
			break;

			case pantallaSubMenuColectoresFil: printScreen(subMenuColectores);
			break;

			case pantallaSubMenuAlarmas: printScreen(subMenuAlarmas);
			break;

			case pantallaSubMenuHistorial: printScreen(subMenuHistorial);
			break;

			case pantallaSubMenuAcercaDe_part1: printScreen(subMenuAcercaDe1);
			break;

			case pantallaSubMenuAcercaDe_part2: printScreen(subMenuAcercaDe2);
			break;

			case pantallaSubmenuSetTempExt: printScreenSettingTemp(_petFilConv);
			break;

			case pantallaSubMenuSelVelocidadColFil: printScreen(subMenuSelVelocidadColec);
			break;

			default: break;
		}
	}

	if((_petFilConv -> previousSetExtTemp != _petFilConv -> setExtTemp) && (*currentScreen == pantallaSubmenuSetTempExt))
	{
		printSettingTemp(_petFilConv);
	}
}


/*
void updateLCD(PETfilConv *_petFilConv)
{


	if(currentScreen != previousScreen) //si la pantalla actual es distinto a la anterior o si la pantalla actual es la de seteo de temp de extrusor
	{
//		previousScreen = currentScreen;

		switch(currentScreen)
		{
			case pantallaWorkingScreen: printscreenWorking(_petFilConv);
			break;

			case pantallaMainMenu_part1: printScreen(prinMenu1);
			break;

			case pantallaMainMenu_part2: printScreen(prinMenu2);
			break;

			case pantallaSubMenuExtrusores: printScreen(subMenuExtrusores);
			break;

			case pantallaSubMenuColectoresFil: printScreen(subMenuColectores);
			break;

			case pantallaSubMenuAlarmas: printScreen(subMenuAlarmas);
			break;

			case pantallaSubMenuHistorial: printScreen(subMenuHistorial);
			break;

			case pantallaSubMenuAcercaDe_part1: printScreen(subMenuAcercaDe1);
			break;

			case pantallaSubMenuAcercaDe_part2: printScreen(subMenuAcercaDe2);
			break;

			case pantallaSubmenuSetTempExt: printScreenSettingTemp(_petFilConv ->setExtTemp);
			break;

			case pantallaSubMenuSelVelocidadColFil: printScreen(subMenuSelVelocidadColec);
			break;
		}
	}

	if((_petFilConv -> setExtTemp != previousSetTempExt) && (currentScreen == pantallaSubmenuSetTempExt))
	{
		previousSetTempExt = _petFilConv -> setExtTemp;
		printSettingTemp(_petFilConv ->setExtTemp);
	}
}

*/

void printScreenSettingTemp(PETfilConv *_petFilConv)
{
	int reg_lines[LINES] = {
					0x80|0x00,
					0x80|0x40,
					0x80|0x14,
					0x80|0x54
				};

	char *subMenuSelTemp[] = {
	    "Elegir T trabajo:   ",
		"                    ",
		"      / 250 C (Max.)",
		"                    "
	};

	char aux[20];

	clearScreen();
	lcd_send_cmd(reg_lines[0]);
	lcd_send_string(subMenuSelTemp[0]);
	lcd_send_cmd(reg_lines[2]);
	sprintf(aux,"%lu C / 250 C(Max.)",_petFilConv -> previousSetExtTemp);
	lcd_send_string(aux);
}

void printSettingTemp(PETfilConv *_petFilConv)
{
	char aux[3];
	int reg_lines[LINES] = {
					0x80|0x00,
					0x80|0x40,
					0x80|0x14,
					0x80|0x54
				};

	lcd_send_cmd(reg_lines[2]);
	sprintf(aux,"%lu C",_petFilConv -> previousSetExtTemp);
	lcd_send_string(aux);
}

void printscreenWorking(PETfilConv *_petFilConv)
{
	int reg_lines[LINES] = {
					0x80|0x00,
					0x80|0x40,
					0x80|0x14,
					0x80|0x54
				};
	int dir_Al = 0x80|0x20;
/*
	char *screenWorking[] = {
	    "   T act / T trab  ",
		"                   ",
		"                   ",
	    "                   "
	};

	char aux[20];

	clearScreen();
	lcd_send_cmd(reg_lines[0]);
	lcd_send_string(screenWorking[0]);
	lcd_send_cmd(reg_lines[2]);
	sprintf(aux," %lu C  / %lu C   ",_petFilConv -> currentExtTemp ,_petFilConv -> setExtTemp);
	lcd_send_string(aux);
*/
	char aux[20];
	char aux2[5] = "COL1:";
	char aux3[4] = "AL:";

	clearScreen();
	lcd_send_cmd(reg_lines[0]);
	sprintf(aux,"EXT1: %lu / %lu C  ",_petFilConv -> currentExtTemp ,_petFilConv -> setExtTemp);
	lcd_send_string(aux);
	lcd_send_cmd(reg_lines[2]);
	lcd_send_string(aux2);
	lcd_send_cmd(dir_Al);
	lcd_send_string(aux3);
}

void printCurrentTemp(PETfilConv *_petFilConv)
{


	int reg_lines[LINES] = {
					0x80|0x00,
					0x80|0x40,
					0x80|0x14,
					0x80|0x54
				};
/*
	char aux[20];
	lcd_send_cmd(reg_lines[2]);
	sprintf(aux," %lu C  / %lu C   ",_petFilConv -> currentExtTemp ,_petFilConv -> setExtTemp);
	lcd_send_string(aux);
*/
	char aux[20];
	lcd_send_cmd(reg_lines[0]);
	sprintf(aux,"EXT1: %lu / %lu C  ",_petFilConv -> currentExtTemp ,_petFilConv -> setExtTemp);
	lcd_send_string(aux);
}

void updateCursor(uint32_t *_indexes , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col)
{
	  clearAllCursor();

	switch(*currentScreen)
	{
		case pantallaMainMenu_part1: printCursor(_indexes[cont_item_PrinMenu]);
		break;

		case pantallaMainMenu_part2: printCursor(_indexes[cont_item_PrinMenu]);
		break;

		case pantallaSubMenuExtrusores: printCursor(_indexes[cont_item_SubMenuExtrusores]);
		break;

		case pantallaSubMenuColectoresFil:	printCursor(_indexes[cont_item_SubMenuColectores]);
		break;

		case pantallaSubMenuAlarmas:
			printCursor(_indexes[cont_item_SubMenuAlarmas]);
			if(_petFilConv -> lightAlarm_state == true)
			{
				lcd_send_cmd(XC_LINE2_LCD);
				lcd_send_string("X");
			}
			else
			{
				lcd_send_cmd(XC_LINE2_LCD);
				lcd_send_string(" ");
			}

			if(_petFilConv -> soundAlarm_state == true)
			{
				lcd_send_cmd(XC_LINE3_LCD);
				lcd_send_string("X");
			}
			else
			{
				lcd_send_cmd(XC_LINE3_LCD);
				lcd_send_string(" ");
			}
		break;

		case pantallaSubMenuHistorial: printCursor(_indexes[cont_item_SubMenuHistorial]);
		break;

		case pantallaSubMenuSelVelocidadColFil:
			printCursor(_indexes[cont_item_SubMenuSelVelCol]);
			if(*_vel_col == APAGADO)	printX(1);
			else if(*_vel_col == VELOCIDAD_1)	printX(2);
			else if(*_vel_col == VELOCIDAD_2)	printX(3);
			else if(*_vel_col == VELOCIDAD_3)	printX(4);
		break;

		default: break;
	}
}

/*
void updateCursor(void)
{
  if(
		(previousIndexes[cont_item_PrinMenu] != indexes[cont_item_PrinMenu])||	//cambio indice en menu principal?
		(previousIndexes[cont_item_SubMenuExtrusores] != indexes[cont_item_SubMenuExtrusores])||	//cambio indice en submenu Extrusores?
		(previousIndexes[cont_item_SubMenuColectores] != indexes[cont_item_SubMenuColectores])||	//cambio indice en submenu Colectores?
		(previousIndexes[cont_item_SubMenuAlarmas] != indexes[cont_item_SubMenuAlarmas])||	//cambio indice en submenu Alarmas?
		(previousIndexes[cont_item_SubMenuHistorial] != indexes[cont_item_SubMenuHistorial])||	//cambio indice en submenu Historial?
		(previousIndexes[cont_item_SubMenuAcercaDe] != indexes[cont_item_SubMenuAcercaDe])||	//cambio indice en submenu Acerca De...?
		(previousIndexes[cont_item_SubMenuSelVelCol] != indexes[cont_item_SubMenuSelVelCol])||	//cambio indice en submenu Seleccion de velocidad colector?
		(previousScreen != currentScreen)||		//cambio de pantalla?
		changeAlarms == true
	)
  {
	  previousScreen = currentScreen;
	  for(int i = 1 ; i < n_pantallas; i++) previousIndexes[i] = indexes[i];

	  clearAllCursor();

	switch(currentScreen)
	{
//		case pantallaWorkingScreen:	//NO MUESTRA CURSOR
//		break;

		case pantallaMainMenu_part1: printCursor(indexes[cont_item_PrinMenu]);
		break;

		case pantallaMainMenu_part2: printCursor(indexes[cont_item_PrinMenu]);
		break;

		case pantallaSubMenuExtrusores: printCursor(indexes[cont_item_SubMenuExtrusores]);
		break;

		case pantallaSubMenuColectoresFil:	printCursor(indexes[cont_item_SubMenuColectores]);
		break;

		case pantallaSubMenuAlarmas:
			printCursor(indexes[cont_item_SubMenuAlarmas]);
			if(PETfilConv1.lightAlarm_state == true)
			{
				lcd_send_cmd(XC_LINE2_LCD);
				lcd_send_string("X");
			}
			else
			{
				lcd_send_cmd(XC_LINE2_LCD);
				lcd_send_string(" ");
			}

			if(PETfilConv1.soundAlarm_state == true)
			{
				lcd_send_cmd(XC_LINE3_LCD);
				lcd_send_string("X");
			}
			else
			{
				lcd_send_cmd(XC_LINE3_LCD);
				lcd_send_string(" ");
			}

			changeAlarms = false;
		break;

		case pantallaSubMenuHistorial: printCursor(indexes[cont_item_SubMenuHistorial]);
		break;

		case pantallaSubMenuSelVelocidadColFil:
			printCursor(indexes[cont_item_SubMenuSelVelCol]);
			if(PETfilConv1.velocidad_Col == APAGADO)	printX(1);
			else if(PETfilConv1.velocidad_Col == VELOCIDAD_1)	printX(2);
			else if(PETfilConv1.velocidad_Col == VELOCIDAD_2)	printX(3);
			else if(PETfilConv1.velocidad_Col == VELOCIDAD_3)	printX(4);
		break;

//		case pantallaSubMenuAcercaDe_part1: //NO MUESTRA CURSOR
//		break;

//		case pantallaSubmenuSetTempExt: //NO MUESTRA CURSOR
//		break;
		default: break;
	}
  }
}
*/

void printX(uint8_t i)
{
	lcd_send_cmd(XC_LINE1_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE2_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE3_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE4_LCD);
	lcd_send_string(" ");

	if(i == 1)	lcd_send_cmd(XC_LINE1_LCD);
	else if(i == 2)	lcd_send_cmd(XC_LINE2_LCD);
	else if(i == 3)	lcd_send_cmd(XC_LINE3_LCD);
	else if(i == 4)	lcd_send_cmd(XC_LINE4_LCD);
	lcd_send_string("X");
}

void printCursor(uint8_t index)
{
	char *cursor[] = {">"};

	switch(index)
	{
		case 0:	lcd_send_cmd(C_LINE1_LCD);	//primera linea
		break;

		case 1:	lcd_send_cmd(C_LINE2_LCD);	//segunda linea
		break;

		case 2:	lcd_send_cmd(C_LINE3_LCD);	//tercera linea
		break;

		case 3:	lcd_send_cmd(C_LINE4_LCD);	//cuarta linea
		break;

		case 4:	lcd_send_cmd(C_LINE1_LCD);	//primer linea
			break;

		case 5:	lcd_send_cmd(C_LINE2_LCD);	//segunda linea
			break;

		case 6:	lcd_send_cmd(C_LINE3_LCD);	//tercera linea
			break;

		default: break;
	}

	lcd_send_string(cursor[0]);
}

void clearAllCursor(void)
{
	char *char_vacio[] = {" "};

	lcd_send_cmd(C_LINE1_LCD);	//primer linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE2_LCD);	//segunda linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE3_LCD);	//tercera linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE4_LCD);	//cuarta linea
	lcd_send_string(char_vacio[0]);
}

void printScreen(char *screen[])
{
	int reg_lines[LINES] = {
					0x80|0x00,
					0x80|0x40,
					0x80|0x14,
					0x80|0x54
				};

	clearScreen();
	  for(int i = 0 ; i < LINES ; i++)
	  {
		  lcd_send_cmd(reg_lines[i]);
		  lcd_send_string(screen[i]);
	  }
}

void clearScreen(void)
{
	  lcd_send_cmd (CLEAR_LCD);  // clear display
	  HAL_Delay(100);
}

// *********** funciones para manejo de MOTOR 1 *****************

void ControlMotor(PETfilConv *_petFilConv)
{
	if(_petFilConv -> col_state)	HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_SET);	//si RESET_MOTOR == 1, el motor se habilita
	else						HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_RESET);	//si RESET_MOTOR == 0, el motor se deshabilita
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t previousVel_Col1 = APAGADO;

	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 5-1 (VELOCIDAD 3: 500 mm/min)
	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 6-1 (VELOCIDAD 2: 400 mm/min)
	//Fclk = 56MHz / prescaler = 56000-1 / autoreload = 7-1 (VELOCIDAD 1: 300 mm/min)

	if(htim->Instance == TIM2)
	{
		if(g_velocidad_Col1 != previousVel_Col1)
		{
			switch(g_velocidad_Col1)
			{
				case VELOCIDAD_1:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 7-1);
					HAL_TIM_Base_Start_IT(htim);
					break;
				case VELOCIDAD_2:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 6-1);
					HAL_TIM_Base_Start_IT(htim);
					break;
				case VELOCIDAD_3:
					HAL_TIM_Base_Stop_IT(htim);
					__HAL_TIM_SET_COUNTER(htim, 0);
					__HAL_TIM_SET_AUTORELOAD(htim, 5-1);
					HAL_TIM_Base_Start_IT(htim);
					break;

				default: break;
			}

			previousVel_Col1 = g_velocidad_Col1;
		}

		HAL_GPIO_TogglePin(GPIOA, PASOS_MOTOR1_Pin);
	}
}

// *********** funciones para manejo de PID 1 *****************

float_t getTemp(void)
{
	//	float_t termistorRes = 0;
	//	float_t steinhart = 0;
		static float_t factor_cor2 = 10;
		static float_t Temp_C = 0;
		float_t adc_val_average = 0;
		uint32_t adc_val = 0;

		for(uint8_t i = 0 ; i < N_SAMPLES_ADC ; i++)
		{
			HAL_ADC_Start_DMA(&hadc1, &adc_val, 1);	//star in DMA mode and we are reading only 1 channel or 1 word
			adc_val_average += adc_val;
		}
		adc_val_average = adc_val_average/N_SAMPLES_ADC;	//promedio

	/*
		termistorRes = (adc_val_average * 3.3)/(4095 - adc_val_average);
		steinhart = termistorRes / termistorNominalRes;
		steinhart = log(steinhart);
		steinhart /= termistorBValue;
		steinhart += 1.0 / (termistorNominalTemp + 273.15);
		steinhart = 1.0 / steinhart;
		steinhart -= 273.15;

		return	steinhart;

			Vout = adc_val_average * (3.3/4095);
			R_NTC = (Vout * 100000) / (Vsupply - Vout);
			Temp_K = (T0 * B_param) / (T0 * log(R_NTC / 130000) + B_param);
			Temp_C = Temp_K - 273.15;
			return	Temp_C;
	*/

			while(tempNTC100K[i].adc < adc_val_average) i++;

			if(i == NUMTEMPS)	Temp_C = tempNTC100K[i-1].temp;
			else if(i == 0) Temp_C = tempNTC100K[i].temp;
			else
			{
				Temp_C = tempNTC100K[i-1].temp + (adc_val_average - tempNTC100K[i-1].adc) * (float)(tempNTC100K[i].temp - tempNTC100K[i-1].temp) / (float)(tempNTC100K[i].adc - tempNTC100K[i-1].adc);

				//aux = (tempNTC100K[i].temp - tempNTC100K[i-1].temp) / ( tempNTC100K[i].adc - tempNTC100K[i-1].adc);
				//tempC = aux * (adc_val - tempNTC100K[i].adc) + tempNTC100K[i].temp;
			}

			//tempCint = (uint32_t)Temp_C;
			return	(Temp_C - factor_cor2);
}

void Control_ON_OFF(PETfilConv *_petFilConv)
{
	static uint32_t previousTemp = 0;
	//static uint32_t PID_value = 0;
	//char aux[10];

	_petFilConv -> currentExtTemp = (uint32_t)getTemp();

	if(previousTemp != _petFilConv -> currentExtTemp)
	{
		previousTemp = _petFilConv -> currentExtTemp;
		if((_petFilConv -> currentExtTemp) < ((_petFilConv -> setExtTemp) - 3))	HAL_GPIO_WritePin(GPIOB, CALENTADOR1_Pin, GPIO_PIN_RESET); //PID_value = 255;
		else	HAL_GPIO_WritePin(GPIOB, CALENTADOR1_Pin, GPIO_PIN_SET);	//PID_value = 0;
		//htim1.Instance->CCR1 = 255 - PID_value;
		//updateLCD();
//		lcd_send_cmd(0x80|0x54);
//		sprintf(aux," %lu C     ",PID_value);
//		lcd_send_string(aux);
	}
}

/*
void controlPID(PETfilConv *_petFilConv)
{
	float_t PID_value;
	float_t temperature_read = getTemp();  //esta variable vendria a ser la que guarda el resultado de getTemp()

	//tempConv = (uint32_t)temperature_read;
	_petFilConv -> currentExtTemp = (uint32_t)temperature_read;
//	updateLCD();

	//PID_error = set_temperature - temperature_read; //Next we calculate the error between the setpoint and the real value
	//PID_error = set_temperature - temperature_read + factor_cor; //Next we calculate the error between the setpoint and the real value. Este "20" se debe a una corrección que tengo que hacer para que mida bien.
	_petFilConv -> PID_error = _petFilConv -> setExtTemp - temperature_read + _petFilConv -> factor_cor; //Next we calculate the error between the setpoint and the real value. Este "20" se debe a una corrección que tengo que hacer para que mida bien.
	_petFilConv -> PID_p = kp * (_petFilConv -> PID_error);   //Calculate the P value
	//PID_p = 0.01 * kp * PID_error;   //Calculate the P value
	if((-3 < _petFilConv -> PID_error ) && ( _petFilConv -> PID_error < 3 ))
	{
		//PID_i = PID_i + (ki * PID_error); //Calculate the I value in a range on +-3
		_petFilConv -> PID_i = _petFilConv -> PID_i + (ki * (_petFilConv -> PID_error)); //Calculate the I value in a range on +-3
		//PID_i = 0.01 * PID_i + (ki * PID_error); //Calculate the I value in a range on +-3
	}

	//For derivative we need real time to calculate speed change rate
	_petFilConv -> timePrev = _petFilConv -> time; // the previous time is stored before the actual time read
	_petFilConv -> time = HAL_GetTick();	// actual time read
	_petFilConv -> elapsedTime = (_petFilConv -> time - _petFilConv -> timePrev) / 1000;
	//PID_d = kd * ((PID_error - previous_error)/elapsedTime); //Now we can calculate the D value
	_petFilConv -> PID_d = kd * ((_petFilConv -> PID_error - _petFilConv -> previous_error)/_petFilConv -> elapsedTime); //Now we can calculate the D value
	//PID_d = 0.01 * kd * ((PID_error - previous_error)/elapsedTime); //Now we can calculate the D value
	PID_value =(uint32_t)(_petFilConv -> PID_p + _petFilConv -> PID_i + _petFilConv -> PID_d); //Final total PID value is the sum of P + I + D

	//We define PWM range between 0 and 255
	if(PID_value < 0) PID_value = 0;
	else if(PID_value > 255)  PID_value = 255;
	//updateLCD();
	//Now we can write the PWM signal to the mosfet on digital pin. the mosfet is activated with a LOW value
	htim1.Instance->CCR1 = 255 - PID_value;
	//ESCRIBE EL VALOR DEL PWM

	_petFilConv -> previous_error = _petFilConv -> PID_error;     //Remember to store the previous error for next loop.
}
*/

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
