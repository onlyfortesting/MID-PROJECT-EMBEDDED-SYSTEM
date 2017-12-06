/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stm32f1xx_hal_uart.h"
//#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t counter, jam, menit, detik, lapA, lapB, lapC;
char buf[16];
char kata[125];
char str[100];
uint32_t counter32;

int isStart = 0;
int buttonState1;
int lastButtonState1 = 1;
unsigned long lastDebounceTime1 = 0;

int kondisi2 = 0;
int buttonState2;
int lastButtonState2 = 1;
unsigned long lastDebounceTime2 = 0;

unsigned long debounceDelay = 50;
unsigned long prevtick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char bouncing=0xFF;
uint8_t readPinND(GPIO_TypeDef* Port, uint16_t Pin){		//SERING BOUNCING
	uint8_t flag = 0;
	if(HAL_GPIO_ReadPin(Port,Pin)== GPIO_PIN_RESET){
		bouncing=(bouncing<<1);
	} else {
		bouncing= (bouncing<<1)|1;
	}
	if (bouncing==0x03)
		flag = 1;
	return flag;
}

//	  int reading1 = HAL_GPIO_ReadPin(PB1_GPIO_Port,PB1_Pin);
//	  if(reading1 != lastButtonState1) lastDebounceTime1 = HAL_GetTick();
//	  if (HAL_GetTick() - lastDebounceTime1 > debounceDelay) {
//	    if (reading1 != buttonState1) {
//	      buttonState1 = reading1;
//	      if (buttonState1 == GPIO_PIN_SET) kondisi1 = !kondisi1;
//	    }
//	  }
//	  lastButtonState1 = reading1;

uint8_t readPin(GPIO_TypeDef* Port, uint16_t Pin) {
	uint8_t flag = 0;
	uint32_t Timeout_loop=0;
	uint32_t Timeout_value=0x7FFFFF;
	if (HAL_GPIO_ReadPin(Port, Pin) == GPIO_PIN_RESET) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(Port, Pin)==GPIO_PIN_RESET){
			while(HAL_GPIO_ReadPin(Port, Pin)==GPIO_PIN_RESET && Timeout_loop++ < Timeout_value);
			if(Timeout_loop >= Timeout_value) {
				char strerr[16];
				switch(Pin) {
				case GPIO_PIN_3:
					sprintf(strerr,"sensor-A");
					break;
				case GPIO_PIN_4:
					sprintf(strerr,"sensor-B");
					break;
				case GPIO_PIN_2:
					sprintf(strerr,"sensor-C");
					break;
				case GPIO_PIN_0:
					sprintf(strerr,"button-start");
					break;
				case GPIO_PIN_1:
					sprintf(strerr,"button-reset");
					break;
				}
				sprintf(str,"Error:%s\r\n",strerr);
				HAL_UART_Transmit(&huart3,str,strlen(str),1000);
//				CDC_Transmit_FS(str, strlen(str));
			}
			else {
				flag = 1;
			}
		}
	}
	return flag;
}

void TimerStart() {
	counter++;
	if(counter==100) {
		detik++;
		if(detik==60) {
			menit++;
			detik = 0;
		}
		if(menit==60) {
			jam++;
			menit = 0;
		}
		counter = 0;
	}
}

void TimerReset() {
	isStart = counter = detik = menit = jam = 0;
}

void CounterPlus(uint8_t mobil) {
	if(isStart == 1) {
		switch(mobil) {
		case 1:
			lapA++;
			break;
		case 2:
			lapB++;
			break;
		case 3:
			lapC++;
			break;
		}
	}
//	sprintf(str,"%02d:%02d:%02d  |  A:%2d   B:%2d   C:%2d\r\n",menit,detik,counter,lapA,lapB,lapC);
//	HAL_UART_Transmit(&huart3,str,strlen(str),100);
	sprintf(str,"%d-%d-%d\r\n",lapA,lapB,lapC);
	HAL_UART_Transmit(&huart3,str,strlen(str),100);
}

void CounterReset() {
	lapA = lapB = lapC = 0;
}

void DisplayLCD() {
	lcd_gotoxy(0,0);
	sprintf(buf,"    %02d:%02d:%02d",menit,detik,counter);
	lcd_putstr(buf);
	lcd_gotoxy(0,1);
	sprintf(buf,"A:%02d  B:%02d  C:%02d",lapA,lapB,lapC);
	lcd_putstr(buf);
}

void DisplayInit() {
	lcd_init();
	lcd_gotoxy(0,0);
	lcd_putstr("  PROJECT UTS");
	lcd_gotoxy(0,1);
	lcd_putstr("EMBEDDED SYSTEM");
	HAL_Delay(500);
	lcd_clear();
//	lcd_gotoxy(0,0);
//	lcd_putstr("Bagaskara");
//	lcd_gotoxy(0,1);
//	lcd_putstr("1110151041");
	HAL_Delay(500);
	lcd_clear();
	DisplayLCD();
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  DisplayInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
//	  lapA++;
//	  lapB=100+lapA;
//	  lapC=200+lapA;

//	  if(lapA>=50) lapA=0;

	sprintf(str,"2-3-4-\r\n",lapA,lapB,lapC);
	HAL_UART_Transmit(&huart3,str,strlen(str),100);

	  if(readPin(PB1_GPIO_Port, PB1_Pin)) {
		  isStart = !isStart;
	  }
	  if(readPin(PB2_GPIO_Port, PB2_Pin)) {
		  TimerReset();
		  CounterReset();
	  }
	  if(readPin(PB3_GPIO_Port, PB3_Pin)) {
		  CounterPlus(1);
	  }
	  if(readPin(PBStart_GPIO_Port, PBStart_Pin)) {
		  CounterPlus(2);
	  }
	  if(readPin(PBReset_GPIO_Port, PBReset_Pin)) {
		  CounterPlus(3);
	  }

	  DisplayLCD();


//	  sprintf(kata,"%d\r\n",counter32++);
//	  CDC_Transmit_FS(kata, strlen(kata));

//	  if(HAL_GetTick() - prevtick >= 200) {
//	  		sprintf(kata,"%d:%02d:%02d:%02d A=%d B=%d\r\n",jam,menit,detik,counter,lapA,lapB);
//	  		CDC_Transmit_FS(kata, strlen(kata));
//	  		prevtick = HAL_GetTick();
//	  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1_Pin PB2_Pin PB3_Pin PBStart_Pin 
                           PBReset_Pin */
  GPIO_InitStruct.Pin = PB1_Pin|PB2_Pin|PB3_Pin|PBStart_Pin 
                          |PBReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin RW_Pin EN_Pin */
  GPIO_InitStruct.Pin = RS_Pin|RW_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM2) {
		if(isStart == 1)
			TimerStart();
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
