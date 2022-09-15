/*
 * gui.c
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "gui.h"
#include "lcd.h"
#include "controlTemp.h"
#include "motors.h"
#include "alarms.h"
#include "main.h"

#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES & DEFINES ******
#define MAX_TEMP	275
#define STEP_TEMP	5


//****** FUNCTIONS IMPLEMENTATIONS ******

void UpdateDataGUI(uint32_t *_indexes , uint32_t *_btn , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col)
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
						}
						else if(_indexes[cont_item_SubMenuAlarmas] == 2)
						{
							if(_petFilConv -> soundAlarm_state == false)	_petFilConv -> soundAlarm_state = true;
							else	_petFilConv -> soundAlarm_state = false;
						}
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
						_petFilConv -> flagTemp_state = false;
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
							else _petFilConv -> previousSetExtTemp = _petFilConv -> previousSetExtTemp - 5;
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
							if(_petFilConv -> previousSetExtTemp == MAX_TEMP) _petFilConv -> previousSetExtTemp = MAX_TEMP;
							else _petFilConv -> previousSetExtTemp = _petFilConv -> previousSetExtTemp + STEP_TEMP;	//Temp++
						break;

						default: break;
					}
			}
	}
}

void UpdateCursor(uint32_t *_indexes , PETfilConv *_petFilConv , uint32_t *currentScreen , uint32_t *_vel_col)
{
	  ClearAllCursor();

	switch(*currentScreen)
	{
		case pantallaMainMenu_part1: PrintCursor(_indexes[cont_item_PrinMenu]);
		break;

		case pantallaMainMenu_part2: PrintCursor(_indexes[cont_item_PrinMenu]);
		break;

		case pantallaSubMenuExtrusores: PrintCursor(_indexes[cont_item_SubMenuExtrusores]);
		break;

		case pantallaSubMenuColectoresFil:	PrintCursor(_indexes[cont_item_SubMenuColectores]);
		break;

		case pantallaSubMenuAlarmas:
			PrintCursor(_indexes[cont_item_SubMenuAlarmas]);
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

		case pantallaSubMenuHistorial: PrintCursor(_indexes[cont_item_SubMenuHistorial]);
		break;

		case pantallaSubMenuSelVelocidadColFil:
			PrintCursor(_indexes[cont_item_SubMenuSelVelCol]);
			if(*_vel_col == APAGADO)	PrintX(1);
			else if(*_vel_col == VELOCIDAD_1)	PrintX(2);
			else if(*_vel_col == VELOCIDAD_2)	PrintX(3);
			else if(*_vel_col == VELOCIDAD_3)	PrintX(4);
		break;

		default: break;
	}
}

