/*
 * gui.h
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

#ifndef INC_GUI_H_
#define INC_GUI_H_

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "main.h"

//****** VARIABLES & DEFINES ******

//GUI constants
#define cantItemsPrinMenu 				6
#define cantItemsSubMenuExtrusores 		2
#define cantItemsSubMenuColectores 		2
#define cantItemsSubMenuAlarmas			3
#define cantItemsSubMenuHistorial		3
#define cantItemsSubMenuAcercaDe		2
#define cantItemsSubMenuSelVelCol		4
#define n_pantallas						8

#define cont_item_PrinMenu				1
#define cont_item_SubMenuExtrusores		2
#define cont_item_SubMenuColectores		3
#define cont_item_SubMenuAlarmas		4
#define cont_item_SubMenuHistorial		5
#define cont_item_SubMenuAcercaDe		6
#define cont_item_SubMenuSelVelCol		7

#define pantallaWorkingScreen 				0	//pantalla 0 - pantalla de trabajo
#define pantallaMainMenu_part1 				11	//pantalla 11 - menu principal (parte 1)
#define pantallaMainMenu_part2 				12	//pantalla 12 - menu principal (parte 2)
#define pantallaSubMenuExtrusores 			2	//pantalla 2 - sub menu EXTRUSORES
#define pantallaSubMenuColectoresFil 		3	//pantalla 3 - sub menu COLECTORES fil.
#define pantallaSubMenuSelVelocidadColFil 	31	//pantalla 31 - sub menu selector velocidad colector fil.
#define pantallaSubMenuAlarmas				4	//pantalla 4 - sub menu ALARMAS
#define pantallaSubMenuHistorial			5	//pantalla 5 - sub menu HISTORIAL
#define pantallaSubMenuAcercaDe_part1		6	//pantalla 6 - sub menu ACERCA DE...(parte 1)
#define pantallaSubMenuAcercaDe_part2		7	//pantalla 7 - sub menu ACERCA DE...(parte 2)
#define pantallaSubmenuSetTempExt			8	//pantalla 8 - sub menu Seteo temp ExtrusorX

//****** FUNCTIONS PROTOTYPES ******

void UpdateDataGUI(uint32_t *_indexes , uint32_t *_btn , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col);
void UpdateCursor(uint32_t *_indexes , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col);

#endif /* INC_GUI_H_ */
