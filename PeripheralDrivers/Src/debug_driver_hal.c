/*
 * debug_driver_hal.c
 *
 *  Created on: Nov 10, 2023
 *      Author: namontoy
 */
#include "stdint.h"
#include "stm32f4xx.h"
#include "debug_driver_hal.h"


static uint32_t aux_dwt_ctrl;

/* Activa la herramienta del Trace (Debug) */
void debug_Enable_Trace(void){
	CoreDebug->DEMCR |= (CoreDebug_DEMCR_TRCENA_Msk);
}

/* Desactiva la herramienta del Trace (Debug) */
void debug_Disable_Trace(void){
	CoreDebug->DEMCR &= ~(CoreDebug_DEMCR_TRCENA_Msk);
}

/* Verifica que la funcion del cyclic-counter existe*/
uint8_t debug_Check_Trace_Available(void){
	aux_dwt_ctrl = (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) ;
	if(aux_dwt_ctrl == 0){
		// existe el cycle-counter -> CYCCNT
		return 1;
	}else{
		return 0;
	}
}

/* Activa y desactiva el cyclic-counter,
 * dependiendo del valor de State */
void debug_StartStop_CycleCounter(uint8_t state){
	if(state == CYCCNT_START){
		DWT->CYCCNT = 0;
		DWT->CTRL |= (DWT_CTRL_CYCCNTENA_Msk);
	}else{
		DWT->CTRL &= ~(DWT_CTRL_CYCCNTENA_Msk);
	}

}
