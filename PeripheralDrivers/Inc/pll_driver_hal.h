/*
 * pll_driver_hal.h
 *
 *  Created on: Nov 15, 2023
 *      Author: namontoy
 */

#ifndef PLL_DRIVER_HAL_H_
#define PLL_DRIVER_HAL_H_

#include "stm32f4xx.h"

enum{
	MCO1_PRESCALER_DIV_3 = 0b101,
	MCO1_PRESCALER_DIV_4 = 0b110,
	MCO1_PRESCALER_DIV_5 = 0b111
};

enum{
	MCO1_HSI_CHANNEL = 0,
	MCO1_LSE_CHANNEL,
	MCO1_HSE_CHANNEL,
	MCO1_PLL_CHANNEL
};

enum{
	HSI_CLOCK_CONFIGURED = 0,
	HSE_CLOCK_CONFIGURED,
	PLL_CLOCK_CONFIGURED
};


#define MAIN_CLOCK

/* -- Cabeceras de las funciones publicas del PLL -- */
void pll_Config_100MHz(void);
void pll_Config_MCO1(uint8_t preescalerMCO, uint8_t channelMCO);
uint8_t pll_Get_MainClock(void);


#endif /* PLL_DRIVER_HAL_H_ */
