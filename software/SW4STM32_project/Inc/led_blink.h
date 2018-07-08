/*
 *******************************************************************************
 *  [led_blink.h]
 *  This module manages led blinking.
 *
 *  This program is under the terms of the GPLv3.
 *  https://www.gnu.org/licenses/gpl-3.0.html
 *
 *  Copyright(c) 2018 Keshikan (www.keshikan.net)
 *******************************************************************************
 */

#ifndef LED_BLINK_H_
#define LED_BLINK_H_

#include <device_conf.h>
#include <stm32f0xx_hal.h>
#include <stdbool.h>

#define LED_NUM (5)
#define LED_CNT_THR (5)

extern void ledInit();
extern void ledActivate(uint32_t led_num);
extern void ledMain();
extern void ledIncrementState();

#endif /* LED_BLINK_H_ */
