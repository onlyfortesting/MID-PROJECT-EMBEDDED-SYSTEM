#include "lcd.h"

/* Private functions */
static void LCD_Pins_Init(void);
static void LCD_Write(uint8_t cmd);
static void LCD_Write4bit(uint8_t cmd);
static void enable(void);

void lcd_init(void)
{
	/* LCD Init pins */
	LCD_Pins_Init();

	/* Clear Enable */
	Clr_En;

	/* Delay Power On*/
	HAL_Delay(16);

	/* Step 1 */
	LCD_Write(0x03);
	HAL_Delay(5);

	/* Step 2 */
	LCD_Write(0x03);
	HAL_Delay(5);

	/* Step 3 */
	LCD_Write(0x03);
	HAL_Delay(1);

	/* Next Init */
	LCD_Write(0x02);
	LCD_Write(0x02);
	LCD_Write(0x08);
	LCD_Write(0x00);
	LCD_Write(0x0E);
	LCD_Write(0x00);
	LCD_Write(0x06);
	LCD_Write(0x00);
	LCD_Write(0x01);
/*
	 * Display On/Off Control:
	 * - Bit 3 --> Must be 1
	 * - Bit 2 --> 0 = Display Off, 1 = Display On
	 * - Bit 1 --> 0 = Cursor displayed Off, 1 = Cursor displayed On
	 * - Bit 0 --> 0 = Cursor blink Off, 1 = Cursor blink On
	 */
	LCD_Write(0b00001100);
}

void lcd_putchar(unsigned char c)
{
	Set_RS;
	Clr_RW;

	LCD_Write4bit(c >> 4);
	LCD_Write4bit(c & 0x0F);
}

void lcd_putstr(char *s)
{
	char c, i=0;
	while((c=*(s+(i++)))!=0)
		lcd_putchar(c);
}

void lcd_clear(void)
{
	LCD_Write(0x00);
	LCD_Write(0x01);
	HAL_Delay(50);
}

void lcd_gotoxy(unsigned char cols, unsigned char rows)
{
	LCD_Write(0x80|(cols+(0x40*rows)));
}

static void enable(void)
{
	Set_En;
	HAL_Delay(3);
	Clr_En;
}

static void LCD_Write(uint8_t cmd)
{
	//HAL_Delay(20);
	Clr_RS;
	Clr_RW;

	LCD_Write4bit(cmd >> 4);
	LCD_Write4bit(cmd & 0x0F);
}

static void LCD_Write4bit(uint8_t cmd)
{
	//LCD_Pins_Init();
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (cmd & 0x08));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (cmd & 0x04));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (cmd & 0x02));
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (cmd & 0x01));
	enable();
}

static void LCD_Pins_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();


	  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin */
	  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
	  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOB, RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);


}
