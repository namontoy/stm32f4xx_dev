/*
 * systick_driver_hal.c
 *
 *  Created on: Oct 20, 2023
 *      Author: namontoy
 */

#include "stdint.h"
#include "stm32f4xx.h"
#include "systick_driver_hal.h"

static uint32_t countTicks = 0;

/* cabeceras de las funciones privadas */
static void systick_config_interrupt(Systick_Handler_t *pSystickHandler);

/*
 *
 */
void systick_Config(Systick_Handler_t *pSystickHandler){

	// La variable que cuenta el tiempo inicia en cero
	countTicks = 0;

	/* 1. Asignar el valor al reload */
	pSystickHandler->pSystick->LOAD = pSystickHandler->Systick_Config.Systick_Reload;

	/* 2. Limpiar el valor del CNT del systick */
	pSystickHandler->pSystick->VAL = 0;

	/* 3. Configurar el registro CTRL */
	/* 3a. Asignamos la señal de reloj deseada */
	// Limpiamos la posicion que deseamos escribir
	pSystickHandler->pSystick->CTRL &= ~(1 << SysTick_CTRL_CLKSOURCE_Pos);
	//pSystickHandler->pSystick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk);

	// Asignamos la señal de reloj principal (Fosc) al Systick
	pSystickHandler->pSystick->CTRL |= (1 << SysTick_CTRL_CLKSOURCE_Pos);
	//pSystickHandler->pSystick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk);

	/* 3b. COnfig interrupcion Systick */
	/* Desactivamos las interrupciones globales mientras configuramos el sistema.*/
	__disable_irq();

	// Cargamos la configuracion de la interrupcion
	systick_config_interrupt(pSystickHandler);

	/* Activamos de nuevo las interrupciones globales. */
	__enable_irq();

	// El systick inicia apagado
	systick_SetState(pSystickHandler, SYSTICK_OFF);

}


/*
 *
 */
void systick_SetState(Systick_Handler_t *pSystickHandler, uint8_t newState){
	if(newState == SYSTICK_ON){
		// Borramos el valor que se tenga
		pSystickHandler->pSystick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
		// Configuramos el Systick como ON
		pSystickHandler->pSystick->CTRL |= (SysTick_CTRL_ENABLE_Msk);

	}else{
		// Desactivamos el Systick
		pSystickHandler->pSystick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk);
	}

}

/**
 *
 */
static void systick_config_interrupt(Systick_Handler_t *pSystickHandler){

	if(pSystickHandler->Systick_Config.Systick_IntState == SYSTICK_INT_ENEABLE){
		// Limpiamos la posicion
		pSystickHandler->pSystick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);

		// Activamos la interrupcion
		pSystickHandler->pSystick->CTRL |= (SysTick_CTRL_TICKINT_Msk);

		// Matriculamos la INT en el NVIC
		NVIC_EnableIRQ(SysTick_IRQn);

		// prioridad ??
	}
	else{
		// Desactivamos la Interrupcion del Systick
		pSystickHandler->pSystick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);

		// Removemos la interrupcion del NVIC
		NVIC_DisableIRQ(SysTick_IRQn);
	}

}

uint32_t systick_GetTicks(void){
	return countTicks;
}

/* Esta función debe ser sobre-escrita en el main para que el sistema funcione*/
/**/
__attribute__((weak)) void systick_Callback(void){
	__NOP();
}


/* IRQ para el Systick */
void SysTick_Handler(void){
	// Evaluamos si en efecto se dio la interrupcion del systick
	if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
		// Limpiamos la bandera de la interrupcion
		SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);

		// Esta variable siempre se incrementa mientras
		// el Systick se encuentre activo
		countTicks++;

		systick_Callback();
	}
}

