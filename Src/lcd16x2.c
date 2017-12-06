/*
 * lcd16x2.c
 *
 *  Created on: 17 Nov 2017
 *      Author: root
 */
#include "lcd16x2.h"
#include "stm32f1xx_hal.h"   // tergantung jenis stm32 yang digunakan

void LCD_Enable()
{
	HAL_GPIO_WritePin(EN_Port,EN_Pin,1);
	HAL_Delay(1);
	HAL_GPIO_WritePin(EN_Port,EN_Pin,0);
	HAL_Delay(1);
}

void LCD_Send4Bit(unsigned char Data)
{
	HAL_GPIO_WritePin(D4_Port, D4_Pin, Data&0x01);
	HAL_GPIO_WritePin(D5_Port, D5_Pin, (Data>>1)&0x01);
	HAL_GPIO_WritePin(D6_Port, D6_Pin, (Data>>2)&0x01);
	HAL_GPIO_WritePin(D7_Port, D7_Pin, (Data>>3)&0x01);
}

void LCD_SendCommand(unsigned char command)
{
	LCD_Send4Bit(command>>4);
	LCD_Enable();
	LCD_Send4Bit(command);
	LCD_Enable();
}

void LCD_Clear()
{
	LCD_SendCommand(0x01);
	HAL_Delay(1);
}

void LCD_Init()
{
	LCD_Send4Bit(0x00);
	HAL_GPIO_WritePin(RS_Port, RS_Pin, 0);
	LCD_Send4Bit(0x03);
	LCD_Enable();
	LCD_Enable();
	LCD_Enable();
	LCD_Send4Bit(0x02);
	LCD_Enable();
	LCD_SendCommand(0x28);
	LCD_SendCommand(0x0C);
	LCD_SendCommand(0x06);
	LCD_SendCommand(0x01);
}

void LCD_Gotoxy(unsigned char x,unsigned char y)
{
	unsigned char address;
	if(!y)address=(0x80+x);
	else address=(0xC0+x);
	LCD_SendCommand(address);
}

void LCD_PutChar(unsigned char Data)
{
	HAL_GPIO_WritePin(RS_Port, RS_Pin, 1);
	LCD_SendCommand(Data);
	HAL_GPIO_WritePin(RS_Port, RS_Pin, 0);
}

void LCD_Puts(char *s)
{
	while(*s){
		LCD_PutChar(*s);
		s++;
	}
}

void sPrintf(char *s,unsigned long bin){
	unsigned char n=0;
	unsigned long backup=bin;
	do{	bin/=10;
		n++;
	}while(bin!=0);

	s+=n;
	*s=' ';                                               //warning bukan masalah biarkan
	while(n--){
		*--s=(backup % 10)+'0';
		backup/=10;
	}
}
