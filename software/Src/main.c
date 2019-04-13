
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "device_conf.h"
#include "curemisc.h"
#include "curebuffer.h"
#include "usbd_midi_if.h"
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

TIM_HandleTypeDef htim2;

DAC_HandleTypeDef hdac;


uint8_t uart_tx_dat;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void USER_TIM2_Init(void);

static void MX_DAC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void dfu_otter_bootloader(void)
{
  *((unsigned long *)0x20003FF0) = 0xDEADBEEF;
  NVIC_SystemReset();
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t curPeriode0 = 0;
uint32_t curPeriode1 = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM14)
	{
		//HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);
    if (TIM2->CNT == 0) {
      TIM2->ARR = (uint32_t)(curPeriode0 / 5); // Play first channel
    	TIM2->CR1 = TIM2->CR1 | 1;
    }
	}
	if(htim->Instance == TIM15)
	{
		//HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);
    if (TIM2->CNT == 0) {
    	TIM2->ARR = (uint32_t)(curPeriode1 / 10); // Play second channel with less power
    	TIM2->CR1 = TIM2->CR1 | 1;
    }
	}
}

/**
  * @brief  The application entry point.
  *
  * @retval None
  */


uint8_t midiBuffer[4];
uint8_t midiState = 0;

uint16_t curTone0 = 0;
uint16_t curTone1 = 0;
uint16_t curChannel = 0;
uint16_t lastTone0 = 0;
uint16_t lastTone1 = 0;
uint32_t noteTimeout = 0;

uint16_t freqs[16] = {0};


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_DAC_Init();

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	MX_TIM14_Init();
	HAL_TIM_Base_Start_IT(&htim14);
	TIM14->CR1 &= ~(1UL);

	MX_TIM15_Init();
	HAL_TIM_Base_Start_IT(&htim15);
	TIM15->CR1 &= ~(1UL);

	USER_TIM2_Init();

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE BEGIN 2 */

  //USB-MIDI Init
  MX_USB_MIDI_INIT();


  if(FUNC_ERROR == midiInit() ){
	  while(1){
		  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
		  HAL_Delay(500);
		  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, RESET);
		  HAL_Delay(500);
	  }
  }

  //Wait usb configuration.
  while(1){
	  if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state){
		  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
		  break;
	  }else{
		  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, RESET);
	  }
  }

  while (1)
  {
  //Wait USB configuration when USB connection error has occurred.
	  while(1){
			if (HAL_GPIO_ReadPin(BUTTON_GPIO, BUTTON_PIN)) {
				dfu_otter_bootloader();
			}
				//HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
			if(USBD_STATE_CONFIGURED == hUsbDeviceFS.dev_state){
			  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
			  break;
		  }else{
			  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, RESET);
			  HAL_Delay(200);
		  }
	  }

		curTone0 = 0;
		curTone1 = 0;

	  for( int i = 0; i < 16; i++ ) { // find max pitch from all channels
	      if( freqs[i] > curTone0 ) {
	          curTone1 = curTone0;
	          curTone0 = freqs[i];
	      }
	      else if( freqs[i] > curTone1 ) {
	          curTone1 = freqs[i];
	      }
	  }

		if (curTone0 > 20 && curTone0 != lastTone0) { // play one tone using TIM14
			curPeriode0 = (uint32_t)1000000 / (uint32_t)(curTone0);
			TIM14->CNT = 0;
			TIM14->ARR = curPeriode0;
			TIM14->CR1 = TIM14->CR1 | 1;
			lastTone0 = curTone0;
			noteTimeout = HAL_GetTick();

			HAL_GPIO_WritePin(LED_FAULT_GPIO, LED_FAULT_PIN, SET);
		} else if (curTone0 < 20 && curTone0 != lastTone0) {
			TIM14->CR1 &= ~(1UL);
      curPeriode0 = 0;

			HAL_GPIO_WritePin(LED_FAULT_GPIO, LED_FAULT_PIN, RESET);
		}

		if (curTone1 > 20 && curTone1 != lastTone1) { // play other polyphonic tone ussing TIM15
			curPeriode1 = (uint32_t)1000000 / (uint32_t)(curTone1);
			TIM15->CNT = 0;
			TIM15->ARR = curPeriode1;
			TIM15->CR1 = TIM15->CR1 | 1;
			lastTone1 = curTone1;
			noteTimeout = HAL_GetTick();
		} else if (curTone1 < 20 && curTone1 != lastTone1) {
			TIM15->CR1 &= ~(1UL);
      curPeriode1 = 0;
		}

		if ((HAL_GetTick() - noteTimeout) > 1000) {
			TIM14->CR1 &= ~(1UL);
			TIM15->CR1 &= ~(1UL);
			for (int i = 0; i < 16; i++) {
				freqs[i] = 0;
			}

			HAL_GPIO_WritePin(LED_FAULT_GPIO, LED_FAULT_PIN, RESET);
		}


		//[USB-MIDI IN] to [MIDI JACK OUT]

		if( FUNC_SUCCESS == midiGetFromUsbRx(0, &uart_tx_dat)){
			if (uart_tx_dat >> 7 == 1) {
				midiBuffer[0] = uart_tx_dat;
				midiBuffer[1] = 0;
				midiBuffer[2] = 0;
				midiState = 1;
			} else if (midiState == 1) {
				midiBuffer[1] = uart_tx_dat;
				midiState = 2;
			} else if (midiState == 2) {
				midiBuffer[2] = uart_tx_dat;
				midiState = 3;

				if ((midiBuffer[0] & 0xF0) == 0x90) { // Note on, 2 data bytes
					char key = midiBuffer[1];
					char vel = midiBuffer[2];

					uint16_t freq = pow(2,(key-0x45)/12.0)*440.0;
					curChannel = midiBuffer[0] & 0xF;
					freqs[curChannel] = freq;


				} if ((midiBuffer[0] &0xF0) == 0x80) { // Note off, 2 data bytes
					char key = midiBuffer[1];
					char vel = midiBuffer[2];

					uint16_t freq = pow(2,(key-0x45)/12.0)*440.0;

					//if (freqs[midiBuffer[0] & 0xF] == freq) {
					freqs[midiBuffer[0] & 0xF] = 0;
					//}
				}
			}
		}

	//[MIDI JACK IN] to [USB-MIDI OUT]
	//midiProcess();


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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


/* TIM14 init function */
static void MX_TIM14_Init(void)
{
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 11;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim14.Init.RepetitionCounter = 0;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 11;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

static void USER_TIM2_Init(void) {
	__HAL_RCC_TIM2_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 3200;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	HAL_TIM_Base_Init(&htim2);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

	HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE);
	HAL_TIM_MspPostInit(&htim2);
}

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/**
  * Enable DMA controller clock
  */


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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
