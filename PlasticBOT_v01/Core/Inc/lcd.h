
//****** INCLUDES ******

#include "stm32f1xx_hal.h"
#include "gui.h"
#include "stdbool.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"

//****** VARIABLES & DEFINES ******

#define SLAVE_ADDRESS_LCD 	0x4E
#define	CLEAR_LCD			0x01
#define LINE1_LCD 			0x80|0x00
#define LINE2_LCD			0x80|0x40
#define LINE3_LCD			0x80|0x14
#define LINE4_LCD			0x80|0x54
#define reg_Al_V			0x80|0x54
#define reg_Al_S			0x80|0x5A
#define C_LINE1_LCD			0x80|0x13	// direccion para imprimir cursor en linea 1
#define XC_LINE1_LCD		0x80|0x11	// direccion para imprimir X en linea 1
#define C_LINE2_LCD			0x80|0x53	// direccion para imprimir cursor en linea 2
#define XC_LINE2_LCD		0x80|0x51	// direccion para imprimir X en linea 2
#define C_LINE3_LCD			0x80|0x27	// direccion para imprimir cursor en linea 3
#define XC_LINE3_LCD		0x80|0x25	// direccion para imprimir X en linea 3
#define C_LINE4_LCD			0x80|0x67	// direccion para imprimir cursor en linea 4
#define XC_LINE4_LCD		0x80|0x65	// direccion para imprimir X en linea 4
#define LINES				4


//****** FUNCTIONS PROTOTYPES ******

bool lcd_init (void);   // initialize lcd
void lcd_send_cmd (char cmd);  // send command to the lcd
void lcd_send_data (char data);  // send data to the lcd
void lcd_send_string (const char *str);  // send string to the lcd
void lcd_clear (void);  
void PresentacionLCD(void);
void UpdateLCD(PETfilConv *_petFilConv , uint32_t *currentScreen);
void PrintScreenSettingTemp(PETfilConv *_petFilConv);
void PrintSettingTemp(PETfilConv *_petFilConv);
void PrintScreenWorking(PETfilConv *_petFilConv);
void UpdateCurrentTemp(PETfilConv *_petFilConv);
void PrintX(uint8_t i);
void CleanX(void);
void PrintCursor(uint8_t index);
void ClearAllCursor(void);
void PrintScreen(const char *screen[]);
void ClearScreen(void);
void PrintErrorMessage(void);
