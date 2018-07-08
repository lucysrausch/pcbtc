/*
 *******************************************************************************
 *  [led_blink.c]
 *  This module manages led blinking.
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2018 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#include <led_blink.h>

uint32_t cnt[LED_NUM];
bool isActivate[LED_NUM];

GPIO_TypeDef* led_gpio[LED_NUM]={LED_IN1_GPIO, LED_IN2_GPIO, LED_OUT1_GPIO, LED_OUT2_GPIO, LED_POW_GPIO};

uint16_t led_pin[LED_NUM]={LED_IN1_PIN, LED_IN2_PIN, LED_OUT1_PIN, LED_OUT2_PIN, LED_POW_PIN};

//call before calling other methods.
void ledInit()
{
	for(uint32_t i=0; i<LED_NUM; i++){
		cnt[i] = 0;
		isActivate[i] = false;
	}
}

void ledActivate(uint32_t led_num)
{
	if(led_num >= LED_NUM){
		return;
	}

	isActivate[led_num] = true;
	HAL_GPIO_WritePin(led_gpio[led_num], led_pin[led_num], SET);
}

// Call from timer
void ledIncrementState()
{
	for(uint32_t i=0; i<LED_NUM; i++){
		if(isActivate[i]){
			cnt[i]++;
		}
	}
}

// Call from main()
void ledMain()
{
	for(uint32_t i=0; i<LED_NUM; i++){
		if(cnt[i] >= LED_CNT_THR){
			isActivate[i] = false;
			cnt[i] = 0;
			HAL_GPIO_WritePin(led_gpio[i], led_pin[i], RESET);
		}
	}
}

