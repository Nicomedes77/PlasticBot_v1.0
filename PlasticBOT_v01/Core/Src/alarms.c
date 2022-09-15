/*
 * alarms.c
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */
//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "alarms.h"
#include "controlTemp.h"
#include "motors.h"
#include "lcd.h"
#include "gui.h"
#include "main.h"


#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"


//****** FUNCTIONS IMPLEMENTATIONS ******

void ControlAlarms(bool *_sensorFil , PETfilConv *_petFilConv)
{
	if(*_sensorFil)
	{
		if(_petFilConv -> lightAlarm_state)	ActivaAlarmaLuminica();
		if(_petFilConv -> soundAlarm_state)	ActivaAlarmaSonora();
	}

	else	DesactivaAlarmas();
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
