/*
 * LCD_HD44780.h
 *
 *  Created on: Apr 15, 2017
 *      Author: IBNU
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f1xx_hal.h"

#define RS_Pin GPIO_PIN_12
#define RS_GPIO_Port GPIOB
#define RW_Pin GPIO_PIN_13
#define RW_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOA
#define D5_Pin GPIO_PIN_9
#define D5_GPIO_Port GPIOA
#define D6_Pin GPIO_PIN_10
#define D6_GPIO_Port GPIOA
 #define D7_Pin GPIO_PIN_11
#define D7_GPIO_Port GPIOA

/* Deklarasi Pin Control */
#define Set_En		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
#define Clr_En		HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
#define Set_RW		HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_SET);
#define Clr_RW		HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, GPIO_PIN_RESET);
#define Set_RS		HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
#define Clr_RS		HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);

/* Global Function Here */
void lcd_init(void);
void lcd_putchar(unsigned char c);
void lcd_putstr(char *s);
void lcd_clear(void);
void lcd_gotoxy(unsigned char col, unsigned char row);


#endif /* LCD_H_ */
