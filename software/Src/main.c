
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

TIM_HandleTypeDef htim17;

TIM_HandleTypeDef htim2;

TIM_HandleTypeDef htim1;

DAC_HandleTypeDef hdac;

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

uint16_t adcBuffer[3];


uint8_t uart_tx_dat;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void USER_TIM2_Init(void);
static void USER_TIM1_Init(void);

static void MX_DAC_Init(void);

static void MX_DMA_Init(void);
static void MX_ADC_Init(void);

void htim17_update(void);

uint32_t Vout = 0;
uint16_t Iout = 0;
uint32_t Vset = 0;
uint32_t Vmax = 52000;

void htim17_update() {
  Iout = adcBuffer[0] * 1.33f;
  Vout = adcBuffer[1] * 16.6f;
  int32_t error = Vset - Vout;

  TIM1->CCR1 = CLAMP(error / 3, 0, 500);

  if (Vout > 55000) {
    TIM1->CCR1 = 0;
  }
}

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
      TIM2->ARR = (uint32_t)(curPeriode0 / CLAMP(((Iout/200)-10), 3, 16)); // Play first channel
    	TIM2->CR1 = TIM2->CR1 | 1;
    }
	}
	if(htim->Instance == TIM15)
	{
		//HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);
    if (TIM2->CNT == 0) {
    	TIM2->ARR = (uint32_t)(curPeriode1 / 8); // Play second channel with less power
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
  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
  HAL_GPIO_WritePin(LED_FAULT_GPIO, LED_FAULT_PIN, SET);

	MX_DAC_Init();

	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	MX_TIM14_Init();
	HAL_TIM_Base_Start_IT(&htim14);
	TIM14->CR1 &= ~(1UL);

	MX_TIM15_Init();
	HAL_TIM_Base_Start_IT(&htim15);
	TIM15->CR1 &= ~(1UL);

  MX_DMA_Init();
  MX_ADC_Init();

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcBuffer, 3);


	USER_TIM1_Init();

  //HAL_TIM_PWM_Start(&htim1);
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);

  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000);

  USER_TIM2_Init();

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_OnePulse_Start(&htim2, TIM_CHANNEL_2);

  /* USER CODE BEGIN 2 */

  //USB-MIDI Init
  MX_USB_MIDI_INIT();

  //HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, RESET);


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
		  //HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, SET);
		  break;
	  }else{
		  HAL_GPIO_WritePin(LED_POW_GPIO, LED_POW_PIN, RESET);
	  }
  }

  MX_TIM17_Init();
	HAL_TIM_Base_Start_IT(&htim17);

  for (uint32_t i = 0; i < Vmax; i+=100) {
    Vset = i;
    HAL_Delay(3);
  }

  HAL_GPIO_WritePin(LED_FAULT_GPIO, LED_FAULT_PIN, RESET);

  Vset = Vmax;

  while (1)
  {
  //Wait USB configuration when USB connection error has occurred.
	   while(1){
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
    if (HAL_GPIO_ReadPin(BUTTON_GPIO, BUTTON_PIN)) {
      dfu_otter_bootloader();
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
      lastTone0 = 0;

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
      lastTone1 = 0;
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

					// Note on with a velocity of 0x00 is the same as a note off message. MIDI Spec: 
					// "A receiver must be capable of recognizing either method of turning off a note, and should treat them identically."
					if (vel == 0) {
						if (freqs[curChannel] == freq)
							freqs[curChannel] = 0;
					} else {
						freqs[curChannel] = freq;
					}
					
				}

				if ((midiBuffer[0] & 0xF0) == 0x80) { // Note off, 2 data bytes
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

/* ADC init function */
static void MX_ADC_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

/* TIM17 init function */
static void MX_TIM17_Init(void)
{
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 11;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1000;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
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

static void USER_TIM1_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1024;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);
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
