/*
 * lcd.c
 *
 *  Created on: 27 ago. 2022
 *      Author: nicolas
 */

//****** INCLUDES ******
#include "stm32f1xx_hal.h"
#include "lcd.h"
#include "gui.h"
#include "controlTemp.h"
#include "motors.h"
#include "alarms.h"
#include "main.h"

#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES, CONSTANTS & DEFINES ******

extern I2C_HandleTypeDef hi2c1;

static const int reg_lines[LINES] = {
				0x80|0x00,
				0x80|0x40,
				0x80|0x14,
				0x80|0x54
			};

static const char *cursor[] = {">"};
static const char *char_vacio[] = {" "};

static const char *screenStart[] = {
		"     PlasticBOT     ",
		"        v0.1        ",
		"                    ",
		"   CFP nro 8 SMATA  "
	};

static const char *prinMenu1[] = {
    "Pantalla Principal ",
    "Extrusores         ",
    "Colectores fil.    ",
    "Alarmas            "
};

static const char *prinMenu2[] = {
    "Historial          ",
    "Acerca de...       ",
    "                   ",
	"                   "
};

static const char *subMenuExtrusores[] = {
    "Volver...          ",
    "Extrusor 1         ",
	"                   ",
	"                   "
};

static const char *subMenuColectores[] = {
    "Volver...          ",
    "Colector fil 1     ",
	"                   ",
	"                   "
};

static const char *subMenuAlarmas[] = {
    "Volver...          ",
    "Luminica           ",
    "Sonora             ",
	"                   "
};

static const char *subMenuHistorial[] = {
    "Volver...          ",
    "Extrusor 1         ",
    "Borrar historial   ",
	"                   "
};

static const char *subMenuAcercaDe1[] = {
    " Proyecto final de  ",
    "       Esp. en      ",
    " Sistemas Embebidos ",
	"    LSE - FiUBA     "
};

static const char *subMenuAcercaDe2[] = {
    "    Ing. Nicolas    ",
    "    Vargas Alice    ",
	"                    ",
	"        2022        "
};

static const char *subMenuSelVelocidadColec[] = {
    "Apagado            ",
    "100 mm/min         ",
	"200 mm/min         ",
	"300 mm/min         "
};

static const char *subMenuSelTemp[] = {
    "Elegir T trabajo:   ",
	"                    ",
	"      / 275 C (Max.)",
	"                    "
};

//****** FUNCTIONS IMPLEMENTATIONS ******

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

bool lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)

	return true;
}

void lcd_send_string (const char *str)
{
	while (*str) lcd_send_data (*str++);
}

void PresentacionLCD(void)
{
	ClearScreen();
	HAL_Delay(100);
	PrintScreen(screenStart);
	HAL_Delay(4000);
	ClearScreen();
}

void UpdateLCD(PETfilConv *_petFilConv , uint32_t *currentScreen)
{
	static uint32_t previousScreen;

	if(*currentScreen != previousScreen) //si la pantalla actual es distinto a la anterior o si la pantalla actual es la de seteo de temp de extrusor
	{
		previousScreen = *currentScreen;

		switch(*currentScreen)
		{
			case pantallaWorkingScreen: PrintScreenWorking(_petFilConv);
			break;

			case pantallaMainMenu_part1: PrintScreen(prinMenu1);
			break;

			case pantallaMainMenu_part2: PrintScreen(prinMenu2);
			break;

			case pantallaSubMenuExtrusores: PrintScreen(subMenuExtrusores);
			break;

			case pantallaSubMenuColectoresFil: PrintScreen(subMenuColectores);
			break;

			case pantallaSubMenuAlarmas: PrintScreen(subMenuAlarmas);
			break;

			case pantallaSubMenuHistorial: PrintScreen(subMenuHistorial);
			break;

			case pantallaSubMenuAcercaDe_part1: PrintScreen(subMenuAcercaDe1);
			break;

			case pantallaSubMenuAcercaDe_part2: PrintScreen(subMenuAcercaDe2);
			break;

			case pantallaSubmenuSetTempExt: PrintScreenSettingTemp(_petFilConv);
			break;

			case pantallaSubMenuSelVelocidadColFil: PrintScreen(subMenuSelVelocidadColec);
			break;

			default: break;
		}
	}

	if((_petFilConv -> previousSetExtTemp != _petFilConv -> setExtTemp) && (*currentScreen == pantallaSubmenuSetTempExt))
	{
		PrintSettingTemp(_petFilConv);
	}
}

void PrintScreenSettingTemp(PETfilConv *_petFilConv)
{
	char aux[20];

	ClearScreen();
	lcd_send_cmd(reg_lines[0]);
	lcd_send_string(subMenuSelTemp[0]);
	lcd_send_cmd(reg_lines[2]);
	sprintf(aux,"%lu  / 275 C (Max.)",_petFilConv -> previousSetExtTemp);
	lcd_send_string(aux);
}

void PrintSettingTemp(PETfilConv *_petFilConv)
{
	char aux[5];

	lcd_send_cmd(reg_lines[2]);
	sprintf(aux,"%lu ",_petFilConv -> previousSetExtTemp);
	lcd_send_string(aux);
}

void PrintScreenWorking(PETfilConv *_petFilConv)
{
	char aux[20];

	ClearScreen();
	lcd_send_cmd(reg_lines[0]);
	//sprintf(aux,"EXT1: %lu / %lu C  ",_petFilConv -> currentExtTemp ,_petFilConv -> setExtTemp);
	sprintf(aux,"EXT1:     / %lu C  ",_petFilConv -> setExtTemp);
	lcd_send_string(aux);

	if(_petFilConv -> col_state == true)
	{
		lcd_send_cmd(reg_lines[2]);
		lcd_send_string("COL1");
	}

	if(_petFilConv -> lightAlarm_state == true)
	{
		lcd_send_cmd(reg_Al_S);
		lcd_send_string("Al:L");
	}

	if(_petFilConv -> soundAlarm_state == true)
	{
		lcd_send_cmd(reg_Al_V);
		lcd_send_string("Al:S");
	}
}

void UpdateCurrentTemp(PETfilConv *_petFilConv)
{
	char aux[5];

	lcd_send_cmd(0x80|0x06);
	sprintf(aux,"%lu ",_petFilConv -> currentExtTemp);
	lcd_send_string(aux);
}

void PrintX(uint8_t i)
{
	CleanX();

	if(i == 1)	lcd_send_cmd(XC_LINE1_LCD);
	else if(i == 2)	lcd_send_cmd(XC_LINE2_LCD);
	else if(i == 3)	lcd_send_cmd(XC_LINE3_LCD);
	else if(i == 4)	lcd_send_cmd(XC_LINE4_LCD);
	lcd_send_string("X");
}

void CleanX(void)
{
	lcd_send_cmd(XC_LINE1_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE2_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE3_LCD);
	lcd_send_string(" ");
	lcd_send_cmd(XC_LINE4_LCD);
	lcd_send_string(" ");
}

void PrintCursor(uint8_t index)
{
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

void ClearAllCursor(void)
{
	lcd_send_cmd(C_LINE1_LCD);	//primer linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE2_LCD);	//segunda linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE3_LCD);	//tercera linea
	lcd_send_string(char_vacio[0]);
	lcd_send_cmd(C_LINE4_LCD);	//cuarta linea
	lcd_send_string(char_vacio[0]);
}

void PrintScreen(const char *screen[])
{
	ClearScreen();
	  for(int i = 0 ; i < LINES ; i++)
	  {
		  lcd_send_cmd(reg_lines[i]);
		  lcd_send_string(screen[i]);
	  }
}

void ClearScreen(void)
{
	  lcd_send_cmd (CLEAR_LCD);  // clear display
	  HAL_Delay(100);
}

void PrintErrorMessage(void)
{
	lcd_send_cmd(LINE1_LCD);
	lcd_send_string("ERROR!");
	lcd_send_cmd(LINE2_LCD);
	lcd_send_string("Reinicie el equipo");
}
