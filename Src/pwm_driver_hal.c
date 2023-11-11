/*
 * PwmDriver.c
 *
 *  Created on: May 14, 2022
 *      Author: namontoy
 */
#include "pwm_driver_hal.h"

/**/
void pwm_Config(PWM_Handler_t *ptrPwmHandler){

	/* 1. Activar la señal de reloj del periférico requerido */
	if(ptrPwmHandler->ptrTIMx == TIM2){
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM3){
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM4){
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
	else if(ptrPwmHandler->ptrTIMx == TIM5){
		RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	}
	else{
		__NOP();
	}

	/* 1. Cargamos la frecuencia deseada */
	pwm_Set_Frequency(ptrPwmHandler);

	/* 2. Cargamos el valor del dutty-Cycle*/
	pwm_Set_DuttyCycle(ptrPwmHandler);

	/* 2a. Estamos en UP_Mode, el limite se carga en ARR y se comienza en 0 */
	ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_DIR;

	/* 3. Configuramos los bits CCxS del registro TIMy_CCMR1, de forma que sea modo salida
	 * (para cada canal hay un conjunto CCxS)
	 *
	 * 4. Además, en el mismo "case" podemos configurar el modo del PWM, su polaridad...
	 *
	 * 5. Y además activamos el preload bit, para que cada vez que exista un update-event
	 * el valor cargado en el CCRx será recargado en el registro "shadow" del PWM */
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{
		// Seleccionamos como salida el canal
		ptrPwmHandler->ptrTIMx->CCMR1 &= ~(TIM_CCMR1_CC1S);

		// Configuramos el canal como PWM
		ptrPwmHandler->ptrTIMx->CCMR1 |= (0b110 << TIM_CCMR1_OC1M_Pos);

		// Activamos la funcionalidad de pre-load
		ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC1PE;

		break;
	}

	case PWM_CHANNEL_2:{
		// Seleccionamos como salida el canal
		ptrPwmHandler->ptrTIMx->CCMR1 &= ~(TIM_CCMR1_CC2S);

		// Configuramos el canal como PWM
		ptrPwmHandler->ptrTIMx->CCMR1 |= (0b110 << TIM_CCMR1_OC2M_Pos);

		// Activamos la funcionalidad de pre-load
		ptrPwmHandler->ptrTIMx->CCMR1 |= TIM_CCMR1_OC2PE;
		break;
	}

	case PWM_CHANNEL_3:{
		// Seleccionamos como salida el canal
		ptrPwmHandler->ptrTIMx->CCMR2 &= ~(TIM_CCMR2_CC3S);

		// Configuramos el canal como PWM
		ptrPwmHandler->ptrTIMx->CCMR2 |= (0b110 << TIM_CCMR2_OC3M_Pos);

		// Activamos la funcionalidad de pre-load
		ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC3PE;
		break;
	}

	case PWM_CHANNEL_4:{
		// Seleccionamos como salida el canal
		ptrPwmHandler->ptrTIMx->CCMR2 &= ~(TIM_CCMR2_CC4S);

		// Configuramos el canal como PWM
		ptrPwmHandler->ptrTIMx->CCMR2 |= (0b110 << TIM_CCMR2_OC4M_Pos);

		// Activamos la funcionalidad de pre-load
		ptrPwmHandler->ptrTIMx->CCMR2 |= TIM_CCMR2_OC4PE;
		break;
	}

	default:{
		break;
	}

	/* 6. Activamos la salida seleccionada */
	pwm_Enable_Output(ptrPwmHandler);

	}// fin del switch-case
}

/**/
void pwm_Start_Signal(PWM_Handler_t *ptrPwmHandler) {
	ptrPwmHandler->ptrTIMx->CR1 |= TIM_CR1_CEN;
}

/**/
void pwm_Stop_Signal(PWM_Handler_t *ptrPwmHandler) {
	ptrPwmHandler->ptrTIMx->CR1 &= ~TIM_CR1_CEN;
}

/**/
void pwm_Enable_Output(PWM_Handler_t *ptrPwmHandler) {
	switch (ptrPwmHandler->config.channel) {
	case PWM_CHANNEL_1: {
		// Activamos la salida del canal 1
		ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC1E;
		break;
	}

	case PWM_CHANNEL_2: {
		// Activamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC2E;
		break;
	}

	case PWM_CHANNEL_3: {
		// Activamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC3E;

		break;
	}

	case PWM_CHANNEL_4: {
		// Activamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER |= TIM_CCER_CC4E;

		break;
	}

	default: {
		break;
	}
	}
}

/**/
void pwm_Disable_Output(PWM_Handler_t *ptrPwmHandler) {
	switch (ptrPwmHandler->config.channel) {
	case PWM_CHANNEL_1: {
		// Desactivamos la salida del canal 1
		ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC1E;
		break;
	}

	case PWM_CHANNEL_2: {
		// Desactivamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC2E;
		break;
	}

	case PWM_CHANNEL_3: {
		// Desactivamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC3E;

		break;
	}

	case PWM_CHANNEL_4: {
		// Desactivamos la salida del canal 2
		ptrPwmHandler->ptrTIMx->CCER &= ~TIM_CCER_CC4E;

		break;
	}

	default: {
		break;
	}
	}
}


/**/
void pwm_Enable_Event(PWM_Handler_t *ptrPwmHandler) {
	switch (ptrPwmHandler->config.channel) {
	case PWM_CHANNEL_1: {
		// Activamos el evento en el canal 1
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC1G;
		break;
	}

	case PWM_CHANNEL_2: {
		// Activamos el evento en el canal 2
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC2G;
		break;
	}

	case PWM_CHANNEL_3: {
		// Activamos el evento en el canal 3
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC3G;

		break;
	}

	case PWM_CHANNEL_4: {
		// Activamos el evento en el canal 4
		ptrPwmHandler->ptrTIMx->EGR |= TIM_EGR_CC4G;

		break;
	}

	default: {
		break;
	}
	}
}

/**/
void pwm_Disable_Event(PWM_Handler_t *ptrPwmHandler) {
	switch (ptrPwmHandler->config.channel) {
	case PWM_CHANNEL_1: {
		// Desactivamos el evento en el canal 1
		ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC1G;
		break;
	}

	case PWM_CHANNEL_2: {
		// Desactivamos el evento en el canal 2
		ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC2G;
		break;
	}

	case PWM_CHANNEL_3: {
		// Desactivamos el evento en el canal 3
		ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC3G;

		break;
	}

	case PWM_CHANNEL_4: {
		// Desactivamos el evento en el canal 4
		ptrPwmHandler->ptrTIMx->EGR &= ~TIM_EGR_CC4G;

		break;
	}

	default: {
		break;
	}
	}
}

/* La frecuencia es definida por el conjunto formado por el preescaler (PSC)
 * y el valor límite al que llega el Timer (ARR), con estos dos se establece
 * la frecuencia.
 * */
void pwm_Set_Frequency(PWM_Handler_t *ptrPwmHandler){

	// Cargamos el valor del prescaler, nos define la velocidad (en ns) a la cual
	// se incrementa el Timer
	ptrPwmHandler->ptrTIMx->PSC = ptrPwmHandler->config.prescaler -1;

	// Cargamos el valor del ARR, el cual es el límite de incrementos del Timer
	// antes de hacer un update y reload.
	ptrPwmHandler->ptrTIMx->ARR = ptrPwmHandler->config.periodo;
}


/**/
void pwm_Update_Frequency(PWM_Handler_t *ptrPwmHandler, uint16_t newFreq){
	ptrPwmHandler->config.periodo = newFreq;

	pwm_Set_Frequency(ptrPwmHandler);
}

/* El valor del dutty debe estar dado en valores de %, entre 0% y 100%*/
void pwm_Set_DuttyCycle(PWM_Handler_t *ptrPwmHandler){

//	uint32_t auxDutty = 0;
//
//	// Verificamos que los valores estan en los limites adecuados.
//	if(ptrPwmHandler->config.duttyCicle > PWM_DUTTY_100_PERCENT){
//		ptrPwmHandler->config.duttyCicle = PWM_DUTTY_100_PERCENT;
//	}
//	else if(ptrPwmHandler->config.duttyCicle < PWM_DUTTY_0_PERCENT){
//		ptrPwmHandler->config.duttyCicle = PWM_DUTTY_0_PERCENT;
//	}

	//auxDutty = (ptrPwmHandler->config.periodo * ptrPwmHandler->config.duttyCicle) / PWM_DUTTY_100_PERCENT;

	// Seleccionamos el canal para configurar su dutty
	switch(ptrPwmHandler->config.channel){
	case PWM_CHANNEL_1:{
		ptrPwmHandler->ptrTIMx->CCR1 = ptrPwmHandler->config.duttyCicle;

		break;
	}

	case PWM_CHANNEL_2:{
		ptrPwmHandler->ptrTIMx->CCR2 = ptrPwmHandler->config.duttyCicle;
		break;
	}

	case PWM_CHANNEL_3:{
		ptrPwmHandler->ptrTIMx->CCR3 = ptrPwmHandler->config.duttyCicle;
		break;
	}

	case PWM_CHANNEL_4:{
		ptrPwmHandler->ptrTIMx->CCR4 = ptrPwmHandler->config.duttyCicle;
		break;
	}

	default:{
		break;
	}

	}// fin del switch-case

}


/**/
void pwm_Update_DuttyCycle(PWM_Handler_t *ptrPwmHandler, uint16_t newDutty){
	ptrPwmHandler->config.duttyCicle = newDutty;

	pwm_Set_DuttyCycle(ptrPwmHandler);
}




