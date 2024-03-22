/*
 * exti_driver_hal.h
 *
 *  Created on: Sep 28, 2023
 *      Author: ingfisica
 */

#ifndef EXTI_DRIVER_HAL_H_
#define EXTI_DRIVER_HAL_H_

#include "stm32f4xx.h"
#include "gpio_driver_hal.h"

enum
{
	EXTERNAL_INTERRUPT_FALLING_EDGE	= 0,
	EXTERNAL_INTERRUPT_RISING_EDGE
};

enum
{
	e_EXTI_PRIORITY_6	= 6,
	e_EXTI_PRIORITY_7,
	e_EXTI_PRIORITY_8,
	e_EXTI_PRIORITY_9,
	e_EXTI_PRIORITY_10,
	e_EXTI_PRIORITY_11,
	e_EXTI_PRIORITY_12,
	e_EXTI_PRIORITY_13,
	e_EXTI_PRIORITY_14,
	e_EXTI_PRIORITY_15
};

typedef struct
{
	GPIO_Handler_t *pGPIOHandler;	// Handler del pin GPIO que lanzara la interrupción
	uint8_t			priority;		// Configura la prioridad de la interrupcion.
	uint8_t			edgeType;		// Se selecciona si se desea un tipo de flanco subiendo o bajando
}EXTI_Config_t;


void exti_Config(EXTI_Config_t *extiConfig);
void exti_Config_Int_Priority(EXTI_Config_t *extiConfig, uint8_t newPriority);
void callback_ExtInt0(void);
void callback_ExtInt1(void);
void callback_ExtInt2(void);
void callback_ExtInt3(void);
void callback_ExtInt4(void);
void callback_ExtInt5(void);
void callback_ExtInt6(void);
void callback_ExtInt7(void);
void callback_ExtInt8(void);
void callback_ExtInt9(void);
void callback_ExtInt10(void);
void callback_ExtInt11(void);
void callback_ExtInt12(void);
void callback_ExtInt13(void);
void callback_ExtInt14(void);
void callback_ExtInt15(void);

#endif /* EXTI_DRIVER_HAL_H_ */
