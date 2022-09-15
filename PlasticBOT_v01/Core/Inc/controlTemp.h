/*
 * controlTemp.h
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

#ifndef INC_CONTROLTEMP_H_
#define INC_CONTROLTEMP_H_

//****** INCLUDES ******

#include "main.h"

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES & DEFINES ******

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

extern ADC_HandleTypeDef hadc1;

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

//****** FUNCTIONS PROTOTYPES ******

float_t GetTemp(void);
void Control_ON_OFF(PETfilConv *_petFilConv);
void ActivaCalentador(void);
void DesactivaCalentador(void);


#endif /* INC_CONTROLTEMP_H_ */
