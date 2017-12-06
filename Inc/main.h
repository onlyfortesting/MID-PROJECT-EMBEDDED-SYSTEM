/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOC
#define PB1_Pin GPIO_PIN_0
#define PB1_GPIO_Port GPIOA
#define PB2_Pin GPIO_PIN_1
#define PB2_GPIO_Port GPIOA
#define PB3_Pin GPIO_PIN_2
#define PB3_GPIO_Port GPIOA
#define PBStart_Pin GPIO_PIN_3
#define PBStart_GPIO_Port GPIOA
#define PBReset_Pin GPIO_PIN_4
#define PBReset_GPIO_Port GPIOA
#define TX3_Pin GPIO_PIN_10
#define TX3_GPIO_Port GPIOB
#define RX3_Pin GPIO_PIN_11
#define RX3_GPIO_Port GPIOB
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

/* USER CODE BEGIN Private defines */
#if !defined  (LSI_VALUE)
  #define LSI_VALUE    ((uint32_t)40000) /*!< Value of the External oscillator in Hz */
#endif
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
