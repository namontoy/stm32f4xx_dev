/*
 * adc_driver_hal.c
 *
 *  Created on: Oct 11, 2023
 *      Author: namontoy
 */

#include "adc_driver_hal.h"
#include "gpio_driver_hal.h"
#include "stm32f4xx.h"
#include "stm32_assert.h"

/* === Headers for private functions === */
static void adc_enable_clock_peripheral(void);
static void adc_set_resolution(ADC_Config_t *adcConfig);
static void adc_set_alignment(ADC_Config_t *adcConfig);
static void adc_set_sampling_and_hold(ADC_Config_t *adcConfig);
static void adc_set_one_channel_sequence(ADC_Config_t *adcConfig);
static void adc_config_interrupt(ADC_Config_t *adcConfig);

/* Variables y elementos que necesita internamente el driver para funcionar adecuadamente */
GPIO_Handler_t 	handlerADCPin = {0};
uint16_t 		adcRawData = 0;

/*
 *
 * */
void adc_ConfigSingleChannel (ADC_Config_t *adcConfig){

	/* 1. Configuramos el PinX para que cumpla la funcion del canal analogo deseado */
	adc_ConfigAnalogPin(adcConfig->channel);

	/* 2. Activamos la senal de reloj para el ADC */
	adc_enable_clock_peripheral();

	//Limpiamos los registros antes de comenzar a configurar
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;

	/*Comenzamos la configuracion de ADC1*/

	/* 3. Resolucion del ADC */
	adc_set_resolution(adcConfig);

	/* 4. Configuramos el modo Scan como desactivado */
	adc_ScanMode(SCAN_OFF);

	/* 5. Configuramos la alineacion de los datos (derecha o izquierda)*/
	adc_set_alignment(adcConfig);

	/* 6. Desactivamos el "continuos mode" */
	adc_StopContinuousConv();

	/* 7. Aca se deberia configurar el sampling...*/
	adc_set_sampling_and_hold(adcConfig);

	/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencias */
	adc_set_one_channel_sequence(adcConfig);

	/* 9. Configuramos el prescaler del ADC en 2:1 (el mas rapido que se puede tener */
	ADC->CCR &= ~ADC_CCR_ADCPRE;

	/* 10. Desactivamos las interrupciones globales */
	__disable_irq();

	/* 11. Configuramos la interrupcion (si se encuentra activa), ademas de inscribir/remover
	 * la interrupcion enel NVIC */
	adc_config_interrupt(adcConfig);

	/* 12. Activamos el moduloADC */
	adc_peripheralOnOFF(ADC_ON);

	/* 13. Activamos las interrupcions globales */
	__enable_irq();
}

/*
 * Enable Clock signal for ADC peripheral
 * */
static void adc_enable_clock_peripheral(void){
	/* Activamos la senal de reloj para el periferico ADC1(bus APB2) */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
}


/*
 * Configures the resolution for the conversion
 * */
static void adc_set_resolution(ADC_Config_t *adcConfig) {

	/* Selecciona uno de los posibles casos y carga la configuracion */
	switch (adcConfig->resolution) {
	case RESOLUTION_12_BIT: {
		ADC1->CR1 &= ~ADC_CR1_RES_0;
		ADC1->CR1 &= ~ADC_CR1_RES_1;
		break;
	}
	case RESOLUTION_10_BIT: {
		ADC1->CR1 |= ADC_CR1_RES_0;
		ADC1->CR1 &= ~ADC_CR1_RES_1;
		break;
	}
	case RESOLUTION_8_BIT: {
		ADC1->CR1 &= ~ADC_CR1_RES_0;
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}
	case RESOLUTION_6_BIT: {
		ADC1->CR1 |= ADC_CR1_RES_0;
		ADC1->CR1 |= ADC_CR1_RES_1;
		break;
	}
	default: {
		break;
	}
	}
}


/*
 * Set the number alignment: left or right
 * */
static void adc_set_alignment(ADC_Config_t *adcConfig){

	if (adcConfig -> dataAlignment  == ALIGNMENT_RIGHT){
		//Alineacion a la derecha (esta es la forma 'natural'
		ADC1 -> CR2 &= ~ADC_CR2_ALIGN;
	}else{
		//Alineacion a la izquierda (para alguno calculos matematicos
		ADC1 -> CR2 |= ADC_CR2_ALIGN;
	}
}


/*
 * Relacionando con el valor del tiempo de carga del capacitor HOLD
 * */
static void adc_set_sampling_and_hold(ADC_Config_t *adcConfig){

	/* Debemos cargar el valor del sampling time para el canal especifico */
	if (adcConfig -> channel < CHANNEL_10){
		ADC1 -> SMPR2 |= (adcConfig -> samplingPeriod << (3* (adcConfig -> channel)));
	}else {
		ADC1 -> SMPR1 |= (adcConfig -> samplingPeriod << (3* (adcConfig -> channel) - 10));
	}
}


/*
 * Configura el numero de elementos en la secuencia (solo un elemento)
 * Configura cual es el canal adquiere la señal ADC
 * */
static void adc_set_one_channel_sequence(ADC_Config_t *adcConfig){

	/* Estamos seleccionando solo 1 elemento en el conteo de la secuencias */
	ADC1->SQR1 = 0;

	// Borramos las posicion que deseamos escribir.
	ADC1->SQR3 &= ~(0b11111 << 0);

	//Asignamos el canal de la conversion a la primera posicion en la secuencia
	ADC1->SQR3 |= (adcConfig->channel << 0);
}


/*
 * Configura el enable de la interrupcion y la activacion del NVIC
 * */
static void adc_config_interrupt(ADC_Config_t *adcConfig){
	if(adcConfig->interrupState == ADC_INT_ENABLE){
		/* 11. Activamos la interrupcion debida a la finalizacion de una canversion EOC */
		ADC1->CR1 |= ADC_CR1_EOCIE;

		/* 11a. Matriculamos la interrupcion en el NVIC */
		__NVIC_EnableIRQ(ADC_IRQn);

	}else{
		/* x11. Desactivamos la interrupcion debida canversion (EOC) */
		ADC1->CR1 &= ~ADC_CR1_EOCIE;

		/* x11b. Removemos la interrupcion en el NVIC */
		__NVIC_DisableIRQ(ADC_IRQn);
	}

}


/*
 * Controla la activación y desactivaion del modulo ADC desde el registro
 * CR2 del adc.
 * */
void adc_peripheralOnOFF(uint8_t state){
	/* Examinamos el estado*/
	if(state == ADC_ON){
		ADC1 -> CR2 |= ADC_CR2_ADON;

	}else{
		ADC1 -> CR2 &= ~ADC_CR2_ADON;
	}

}


/* Enables and disables the Scan mode...
 * Funciona de la mano con la secuencia de varios canales.
 * No es necesario para el caso de solo un canal simple.
 * */
void adc_ScanMode(uint8_t state){

	if(state == SCAN_OFF){
		ADC1 -> CR1 &= ~ADC_CR1_SCAN;

	}else{
		ADC1 -> CR1 |= ADC_CR1_SCAN;
	}
}


/*
 * Funcion que comienza la conversion ADC simple
 * */
void adc_StartSingleConv(void){
	//Iniciamos un ciclo de conversion ADC
	ADC1->CR2 |= ADC_CR2_SWSTART;
}


/*
 * Funcion que comienza la conversion ADC continua
 * */
void adc_StartContinuousConv(void){
	ADC1 -> CR2 |= ADC_CR2_CONT;
	//Iniciamos un ciclo de conversion ADC
	ADC1->CR2 |= ADC_CR2_SWSTART;
}

/*
 * Funcion que comienza la conversion ADC continua
 * */
void adc_StopContinuousConv(void){
	ADC1 -> CR2 &= ~ADC_CR2_CONT;
}


/*
 * Funcion que me retorna el ultimo dato adquirido por la ADC
 * */
uint16_t adc_GetValue(void){
	return adcRawData;
}


/*
 * Esta es la ISR de la interrupcion por conversion ADC
 * */
void ADC_IRQHandler(void){
	// Evaluamos que se dio la interrupcion por conversion ADC
	if (ADC1->SR & ADC_SR_EOC){
		//Leemos el resultado de la conversion ADC y lo cargamos en un valor auxiliar
		adcRawData = ADC1 -> DR;

		// Hacemos el llamado a la funcion que se ejecutara en el main
		adc_CompleteCallback();
	}

}

__attribute__((weak)) void adc_CompleteCallback(void){
	__NOP();
}

/* Con esta funcion configuramos que pin deseamos que funcione como ADC */
void adc_ConfigAnalogPin(uint8_t adcChannel){

	//Con este switch seleccionemos el canal y lo configuramos como analogo
	switch (adcChannel){
	case CHANNEL_0:
	{
		//Es el pin PA0
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_0;
		break;
	}
	case CHANNEL_1:
	{
		//Es el pin PA1
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_1;
		break;
	}
	case CHANNEL_2:
	{
		//Es el pin PA2
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_2;
		break;
	}
	case CHANNEL_3:
	{
		//Es el pin PA3
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_3;
		break;
	}
	case CHANNEL_4:
	{
		//Es el pin PA4
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_4;
		break;
	}
	case CHANNEL_5:
	{
		//Es el pin PA5
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_5;
		break;
	}
	case CHANNEL_6:
	{
		//Es el pin PA6
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_6;
		break;
	}
	case CHANNEL_7:
	{
		//Es el pin PA7
		handlerADCPin.pGPIOx					= GPIOA;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_7;
		break;
	}
	case CHANNEL_8:
	{
		//Es el pin PB0
		handlerADCPin.pGPIOx					= GPIOB;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_0;
		break;
	}
	case CHANNEL_9:
	{
		//Es el pin PB1
		handlerADCPin.pGPIOx					= GPIOB;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_1;
		break;
	}
	case CHANNEL_10:
	{
		//Es el pin PC0
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_0;
		break;
	}
	case CHANNEL_11:
	{
		//Es el pin PC1
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_1;
		break;
	}
	case CHANNEL_12:
	{
		//Es el pin PC2
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_2;
		break;
	}
	case CHANNEL_13:
	{
		//Es el pin PC3
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_3;
		break;
	}
	case CHANNEL_14:
	{
		//Es el pin PC4
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_4;
		break;
	}
	case CHANNEL_15:
	{
		//Es el pin PC5
		handlerADCPin.pGPIOx					= GPIOC;
		handlerADCPin.pinConfig.GPIO_PinNumber 	= PIN_5;
		break;
	}
	default:
	{
		break;
	}
	}

	handlerADCPin.pinConfig.GPIO_PinMode	= GPIO_MODE_ANALOG;
	gpio_Config(&handlerADCPin);
}


/* Configuracion para hacer conversiones en multiples canales y en un orden especifico */
//void adc_ConfigMultichannel (ADC_Config_t *adcConfig, uint8_t numeroDeCanales){
//
//	/* 2. Activamos la senal de reloj para el periferico ADC1(bus APB2) */
//	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
//
//	//Limpiamos los registros antes de comenzar a configurar
//	ADC1->CR1 = 0;
//	ADC1->CR2 = 0;
//
//	/* 6. Desactivamos el "continuos mode" */
//	ADC1 -> CR2 &= ~ADC_CR2_CONT;
//
//	/* 4. Configuramos el modo Scan como activado */
//	ADC1 -> CR1 |= ADC_CR1_SCAN;
//
//	/* 8. Configuramos la secuencia y cuantos elementos hay en la secuencias */
//	ADC1->SQR1 |= (numeroDeCanales - 1) << ADC_SQR1_L_Pos;
//
//	//Ciclo para recorrer cada elemento de arreglo y configurarlo
//
//	for (uint8_t i = 0; i <= numeroDeCanales; i++){
//		/* 1. Configuramos el PinX para que cumpla la funcion del canal analogo deseado */
//		//configAnalogPin(adcConfig->channel);
//		adc_ConfigAnalogPin(adcConfig[i].channel);
//
//		/*Comenzamos la configuracion de ADC1*/
//
//		/* 3. Resolucion del ADC */
//
//		switch (adcConfig[i].resolution)
//		{
//		case RESOLUTION_12_BIT:
//		{
//			ADC1->CR1 &= ~ADC_CR1_RES_0;
//			ADC1->CR1 &= ~ADC_CR1_RES_1;
//			break;
//		}
//		case RESOLUTION_10_BIT:
//		{
//			ADC1->CR1 |= ADC_CR1_RES_0;
//			ADC1->CR1 &= ~ADC_CR1_RES_1;
//			break;
//		}
//		case RESOLUTION_8_BIT:
//		{
//			ADC1->CR1 &= ~ADC_CR1_RES_0;
//			ADC1->CR1 |= ADC_CR1_RES_1;
//			break;
//		}
//		case RESOLUTION_6_BIT:
//		{
//			ADC1->CR1 |= ADC_CR1_RES_0;
//			ADC1->CR1 |= ADC_CR1_RES_1;
//			break;
//		}
//		default:
//		{
//			break;
//		}
//		}
//
//		/* 5. Configuramos la alineacion de los datos (derecha o izquierda)*/
//
//		if (adcConfig[i]. dataAlignment  == ALIGNMENT_RIGHT){
//			//Alineacion a la derecha (esta es la forma 'natural'
//			ADC1 -> CR2 &= ~ADC_CR2_ALIGN;
//		}else{
//			//Alineacion a la izquierda (para alguno calculos matematicos
//			ADC1 -> CR2 |= ADC_CR2_ALIGN;
//		}
//
//		switch(adcConfig[i].channel){
//		case CHANNEL_0: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_0);
//			break;
//		}
//		case CHANNEL_1: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_1);
//			break;
//		}
//		case CHANNEL_2: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_2);
//			break;
//		}
//		case CHANNEL_3: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_3);
//			break;
//		}
//		case CHANNEL_4: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_4);
//			break;
//		}
//		case CHANNEL_5: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_5);
//			break;
//		}
//		case CHANNEL_6: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_6);
//			break;
//		}
//
//		case CHANNEL_7: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_7);
//			break;
//		}
//		case CHANNEL_8: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_8);
//			break;
//		}
//		case CHANNEL_9: {
//			ADC1->SMPR2 |= (adcConfig[i].samplingPeriod << 3*CHANNEL_9);
//			break;
//		}
//		case CHANNEL_10: {
//			ADC1->SMPR1 |= (adcConfig[i].samplingPeriod << 3*(CHANNEL_10 - CHANNEL_10));
//			break;
//		}
//		case CHANNEL_11: {
//			ADC1->SMPR1 |= (adcConfig[i].samplingPeriod << 3*(CHANNEL_11 - CHANNEL_10));
//			break;
//		}
//		default:
//			break;
//		}
//
//		//Asignamos el canal de la conversion a la primera posicion en la secuencia
//		if (numeroDeCanales <= 6){
//			ADC1->SQR3 |= (adcConfig[i].channel <<	i*5);
//		}
//
//	}
//
//
//	/* 9. Configuramos el prescaler del ADC en 2:1 (el mas rapido que se puede tener */
//	ADC->CCR &= ~ADC_CCR_ADCPRE;
//
//	/* 10. Desactivamos las interrupciones globales */
//	__disable_irq();
//
//	/* 11. Activamos la interrupcion debida a la finalizacion de una canversion EOC */
//	ADC1->CR1 |= ADC_CR1_EOCIE;
//
//	/* 11a Interrupción al final de cada conversión de canal  */
//	ADC1->CR2 |= ADC_CR2_EOCS;
//
//	/* 11b. Matriculamos la interrupcion en el NVIC */
//	__NVIC_EnableIRQ(ADC_IRQn);
//
//	/* 12. Activamos el moduloADC */
//	ADC1 -> CR2 |= ADC_CR2_ADON;
//
//	/* 13. Activamos las interrupcions globales */
//	__enable_irq();
//}

/* Configuracion para trigger externo */
//void adc_ConfigTrigger(uint8_t sourceType, PWM_Handler_t *triggerSignal){
//
//	switch (sourceType) {
//	case TRIGGER_AUTO:
//	{
//		/* aca va la configuracion del conversor haciendo siempre conversiones a toda velocidad */
//		break;
//	}
//	case TRIGGER_MANUAL:
//	{
//		/* Se utiliza con "startSingleConv" */
//		/* 6. Desactivamos el "continuos mode" */
//		ADC1 -> CR2 &= ~ADC_CR2_CONT;
//
//		/* 4. Configuramos el modo Scan como activado */
//		ADC1 -> CR1 |= ADC_CR1_SCAN;
//		break;
//	}
//	case TRIGGER_EXT:
//	{
//		if(triggerSignal->ptrTIMx == TIM1){
//
//		}
//		else if(triggerSignal->ptrTIMx == TIM2){
//			switch(triggerSignal->config.channel){
//			case PWM_CHANNEL_2:
//			{
//				// Limpiamos los bits que deseamos configurar
//				ADC1->CR2 &= ~ADC_CR2_EXTEN;
//
//				// Seleccionamos el flanco de subida de la señal externa
//				ADC1->CR2 |= ADC_CR2_EXTEN_0;
//
//				// Limpiamos la posicion para seleccionar el canal
//				ADC1->CR2 &= ~ADC_CR2_EXTSEL;
//
//				// Cargamos la configuracion que deseamos
//				ADC1->CR2 |= (0b0011 << ADC_CR2_EXTSEL_Pos);
//				break;
//			}
//			case PWM_CHANNEL_3:
//			{
//				// Limpiamos los bits que deseamos configurar
//				ADC1->CR2 &= ~ADC_CR2_EXTEN;
//
//				// Seleccionamos el flanco de subida de la señal externa
//				ADC1->CR2 |= ADC_CR2_EXTEN_0;
//
//				// Limpiamos la posicion para seleccionar el canal
//				ADC1->CR2 &= ~ADC_CR2_EXTSEL;
//
//				// Cargamos la configuracion que deseamos
//				ADC1->CR2 |= (0b0100 << ADC_CR2_EXTSEL_Pos);
//				break;
//			}
//			case PWM_CHANNEL_4:
//			{
//				// Limpiamos los bits que deseamos configurar
//				ADC1->CR2 &= ~ADC_CR2_EXTEN;
//
//				// Seleccionamos el flanco de subida de la señal externa
//				ADC1->CR2 |= ADC_CR2_EXTEN_0;
//
//				// Limpiamos la posicion para seleccionar el canal
//				ADC1->CR2 &= ~ADC_CR2_EXTSEL;
//
//				// Cargamos la configuracion que deseamos
//				ADC1->CR2 |= (0b0101 << ADC_CR2_EXTSEL_Pos);
//				break;
//			}
//			default:
//			{
//				break;
//			}
//
//			}
//		}
//		else if(triggerSignal->ptrTIMx == TIM3){
//
//		}
//		else if (triggerSignal->ptrTIMx == TIM4) {
//
//		}
//		else if (triggerSignal->ptrTIMx == TIM5) {
//
//		}
//
//		break;
//	}
//	default:
//	{
//		/* Se utiliza con "startSingleConv" */
//		/* 6. Desactivamos el "continuos mode" */
//		ADC1 -> CR2 &= ~ADC_CR2_CONT;
//
//		/* 4. Configuramos el modo Scan como activado */
//		ADC1 -> CR1 |= ADC_CR1_SCAN;
//		break;
//	}
//	}
//}
//
