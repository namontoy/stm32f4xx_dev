/*
 * systick_driver_hal.h
 *
 *  Created on: Oct 20, 2023
 *      Author: namontoy
 */

#ifndef SYSTICK_DRIVER_HAL_H_
#define SYSTICK_DRIVER_HAL_H_

#include "stm32f4xx.h"

#define MAIN_CLOCK_16MHZ_1MS	16000

enum{
	SYSTICK_OFF = 0,
	SYSTICK_ON
};

enum{
	SYSTICK_INT_DISABLE = 0,
	SYSTICK_INT_ENEABLE
};

/* Estructura que contiene la configuraciónnecesaria para el Systick*/
typedef struct
{
	uint32_t	Systick_Reload;		// Valor reload (para generar 1 ms)
	uint8_t 	Systick_IntState;	// Activa o desactiva el modo interrupción del timer.
} Systick_Config_t;


/* Handler para el Systick*/
typedef struct
{
	SysTick_Type			*pSystick;
	Systick_Config_t		Systick_Config;
} Systick_Handler_t;

void systick_Config(Systick_Handler_t *pSystickHandler);
void systick_SetState(Systick_Handler_t *pSystickHandler, uint8_t newState);
uint32_t systick_GetTicks(void);
void systick_Delay_ms(uint32_t wait_time_ms);

/* Esta función debe ser sobre-escrita en el main para que el sistema funcione*/
void systick_Callback(void);

#endif /* SYSTICK_DRIVER_HAL_H_ */
