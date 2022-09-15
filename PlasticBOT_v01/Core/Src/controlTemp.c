/*
 * controlTemp.c
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "controlTemp.h"
#include "motors.h"
#include "lcd.h"
#include "gui.h"
#include "alarms.h"

#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES, CONSTANTS & DEFINES ******

extern ADC_HandleTypeDef hadc1;
#define N_SAMPLES_ADC 16

//****** FUNCTIONS IMPLEMENTATIONS ******

float_t GetTemp(void)
{
		uint8_t i = 0;
		static float_t factor_cor2 = 10;
		static float_t Temp_C = 0;
		float_t adc_val_average = 0;
		uint32_t adc_val = 0;

		for(uint8_t j = 0 ; j < N_SAMPLES_ADC ; j++)
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
			}

			//tempCint = (uint32_t)Temp_C;
			return	(Temp_C - factor_cor2);
}

void Control_ON_OFF(PETfilConv *_petFilConv)
{
	static uint32_t previousTemp = 0;

	_petFilConv -> currentExtTemp = (uint32_t)GetTemp();

	if(previousTemp != _petFilConv -> currentExtTemp)
	{
		previousTemp = _petFilConv -> currentExtTemp;

		if((_petFilConv -> currentExtTemp) < ((_petFilConv -> setExtTemp) - 4))	ActivaCalentador();
		else		DesactivaCalentador();
	}
}


void ActivaCalentador(void)
{
	HAL_GPIO_WritePin(GPIOA, CALENTADOR1_Pin, GPIO_PIN_RESET);	//Utiliza lógica negativa
}
void DesactivaCalentador(void)
{
	HAL_GPIO_WritePin(GPIOA, CALENTADOR1_Pin, GPIO_PIN_SET);	//Utiliza lógica negativa
}
