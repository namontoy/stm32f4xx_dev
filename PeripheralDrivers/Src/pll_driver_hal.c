/*
 * pll_driver_hal.c
 *
 *  Created on: Nov 15, 2023
 *      Author: namontoy
 */
#include "stm32f4xx.h"
#include "pll_driver_hal.h"


/*
 * Configuración del reloj para que funcione a 100 MHz
 */
void pll_Config_100MHz(void){

	/* Configuramos ahora el PLL, de forma que lo podamos evaluar en el MCO1 sin necesidad de
	 * que sea activado como reloj principal */

	// 1. Seleccionamos el HSI como la señal de entrada para PLL
	RCC->PLLCFGR &= ~(1 << RCC_PLLCFGR_PLLSRC_Pos);

	// 2. Asignamos el valor de PLLM (divisor) para que la señal que entra al PLL sea de 2MHz
	// 2.a Primero borro el psoible valor que se encuentre cargado
	RCC->PLLCFGR &= ~(0b111111 << RCC_PLLCFGR_PLLM_Pos);

	// 2.b Asigno el valor del divisor (8)
	RCC->PLLCFGR |= (8 << RCC_PLLCFGR_PLLM_Pos);

	// 3. La salida del PLL depende del valor VCO y del preescaler PLLP, el cual su minimo valor
	// es de 2, o sea que el VCO debe ser 100 para al final obtener un valor de 100MHz a la salida
	// del sistema
	RCC->PLLCFGR &= ~(0b11 << RCC_PLLCFGR_PLLP_Pos); // El valor 00 nos selecciona la salida div2

	// 4. EL VCO depende de la división de entrada y posteriormente de una multiplicacion la cual
	// es manejada por el PLLN (multiplicador)
	// 4.a Borro el posible valor configurado
	RCC->PLLCFGR &= ~(0xFF << RCC_PLLCFGR_PLLN_Pos);

	// 4.b Cargamos el valor del multiplicador
	RCC->PLLCFGR |= (99 << RCC_PLLCFGR_PLLN_Pos);

	// 5. Cambio el valor VOS (regulador de voltaje) para que se tenga el rendimiento adecuado
	// Esto se debe hacer antes de encender el PLL
	PWR->CR |= (0x3 << PWR_CR_VOS_Pos);

	// 6. Activamos el PLL en el control register del RCC
	RCC->CR |= RCC_CR_PLLON;

	// 6.a Esperamos a que el PLL se cierre (se estabilice)
	while (!(RCC->CR & RCC_CR_PLLRDY)) {
		__NOP();
	}
	__NOP(); // Para hacer el breakpoit y probar...

	// 7. Organizamos los preescalers del hardware, de forma que funcionen a la velocidad maxima
	// permitida
	// 7.a Preescaler del hardware del procesador, el valor en 0 significa sin preescaler
	RCC->CFGR &= ~(0xF << RCC_CFGR_HPRE_Pos);

	// 7.b Preescaler para el APB2 (high speed), el valor en 0 significa sin preescaler
	RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE2_Pos);

	// 7.c Preescaler para el APB1 (low speed). Se debe poner un preescaler de 2:1, para que la velocidad
	// sea la máxima permitida. Primero lo limpio, luego lo configuro
	RCC->CFGR &= ~(0x7 << RCC_CFGR_PPRE1_Pos); // Limpiar
	RCC->CFGR |= (0x4 << RCC_CFGR_PPRE1_Pos);  // Preescaler 2:1

	__NOP(); // Para hacer el breakpoit y probar...

	// 8. Ahora debemos configurar el periodo de espera para leer la memoria flash (Lantency),
	// tal cual como esta descrito en la seccion 3.4.1, tabla 5, pag 45.
	FLASH->ACR &= ~(0xF << FLASH_ACR_LATENCY_Pos); // Limpiamos la posicion
	FLASH->ACR |= (0x3 << FLASH_ACR_LATENCY_Pos); // Configurando 3 WS
	__NOP(); // Para hacer el breakpoit y probar...

	// 8. Ahora falta hacer el cambio del sistema a que funcione en 100 MHz
	RCC->CFGR &= ~(0x3 << RCC_CFGR_SW_Pos);  // Borramos la posicion
	RCC->CFGR |= (0x2 << RCC_CFGR_SW_Pos); // Activamos la señal del PLL como reloj principal

	__NOP(); // Para hacer el breakpoit y probar...
}

/*
 * Función para configurar el PIN MCO (Pin A8) para que funcione con la salida adecuada
 */
void pll_Config_MCO1(uint8_t preescalerMCO, uint8_t channelMCO){
	// Borro el posible valor que pueda tener la configuracion del preescaler del MCO1
	RCC->CFGR &= ~(0b111 << RCC_CFGR_MCO1PRE_Pos);

	// Configuramos el preescaler del MCO en 1:4, de forma que si el HSI es 16MHz, a la salida
	// del MCO debo tener 4MHz
	RCC->CFGR |= (preescalerMCO << RCC_CFGR_MCO1PRE_Pos);

	// Borro el posible valor almacenado en la posicion que se desea modificar
	RCC->CFGR &= ~(0b11 << RCC_CFGR_MCO1_Pos);
	// 5.a Asignamos el PLL al MCO, para poder verificarlo en el OSC
	RCC->CFGR |= (channelMCO << RCC_CFGR_MCO1_Pos);

	__NOP(); // Para hacer el breakpoit y probar...

}

/**/
uint8_t pll_Get_MainClock(void){

	// Seleccion de los dos valores SWS1 y SWS0, que indican cual es la fuente del reloj principal
	uint32_t auxClockState = (RCC->CFGR & 0xC) >> 2;
	return (uint8_t) auxClockState;
}

