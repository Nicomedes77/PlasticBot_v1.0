/*
 * motors.c
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "motors.h"
#include "lcd.h"
#include "gui.h"
#include "controlTemp.h"
#include "alarms.h"
#include "main.h"

#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES, CONSTANTS & DEFINES ******

extern TIM_HandleTypeDef htim2;

//****** FUNCTIONS IMPLEMENTATIONS ******

void ControlMotor(PETfilConv *_petFilConv)
{
	if(_petFilConv -> col_state)	ActivarMotor();
	else						DesactivarMotor();
}

void ActivarMotor(void)
{
	HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_SET);	//si RESET_MOTOR == 1, el motor se habilita
}

void DesactivarMotor(void)
{
	HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_RESET);	//si RESET_MOTOR == 0, el motor se deshabilita
}

void Colector_Init(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_GPIO_WritePin(GPIOA, RESET_MOTOR1_Pin, GPIO_PIN_RESET);	//si RESET_MOTOR == 0, el motor se deshabilita
}
