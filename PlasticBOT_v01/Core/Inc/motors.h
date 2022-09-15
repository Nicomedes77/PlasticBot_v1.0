/*
 * motors.h
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "main.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** FUNCTIONS PROTOTYPES ******

void ControlMotor(PETfilConv *_petFilConv);
void ActivarMotor(void);
void DesactivarMotor(void);
void Colector_Init(void);

#endif /* INC_MOTORS_H_ */
