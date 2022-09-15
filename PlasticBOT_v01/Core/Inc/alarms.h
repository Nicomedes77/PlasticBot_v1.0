/*
 * alarms.h
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

#ifndef INC_ALARMS_H_
#define INC_ALARMS_H_

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "main.h"

//****** FUNCTIONS PROTOTYPES ******

void ControlAlarms(bool *_sensorFil , PETfilConv *_petFilConv);
void ActivaAlarmaLuminica(void);
void ActivaAlarmaSonora(void);
void DesactivaAlarmas(void);

#endif /* INC_ALARMS_H_ */
