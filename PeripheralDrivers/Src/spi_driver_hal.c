/*
 * spi_driver_hal.c
 *
 *  Created on: xx-xx, 2023
 *      Author: namontoy
 *
 * Recordar que debemos configurar los pines GPIO de forma que coincidan
 */

#include <spi_driver_hal.h>

/* === Headers for private functions === */
static void spi_enable_clock_peripheral(SPI_Handler_t pSpiHandler);
static void spi_set_comm_speed(SPI_Handler_t pSpiHandler);
static void spi_set_mode(SPI_Handler_t pSpiHandler);
static void spi_set_comm_type(SPI_Handler_t pSpiHandler);
static void spi_set_direction(SPI_Handler_t pSpiHandler);
static void spi_set_data_size(SPI_Handler_t pSpiHandler);
static void spi_config_interrupt(SPI_Handler_t pSpiHandler);

/**/
void spi_Config(SPI_Handler_t pSpiHandler){
	/* 1. Activamos la señal de reloj del periférico */
	spi_enable_clock_peripheral(pSpiHandler);

	/* 2. Limpiamos el registro de configuración para comenzar de cero */
	pSpiHandler.pSPIx->CR1 = 0x00;

	/* 3. Configuramos la velocidad a la que se maneja el reloj del SPI */
	spi_set_comm_speed(pSpiHandler);

	/* 4. Configuramos el modo */
	spi_set_mode(pSpiHandler);

	/* 5. Configuramos si fullDuplex a solo recepción */
	spi_set_comm_type(pSpiHandler);

	/* 6. Modo de transferencia en MSB-first */
	spi_set_direction(pSpiHandler);

	/* 7. Activamos para que el equipo se comporte como el maestro de la red*/
	pSpiHandler.pSPIx->CR1 |= SPI_CR1_MSTR;

	/* 8. Configuramos el formato del dato (tamaño) para que sea de 8-bit*/
	spi_set_data_size(pSpiHandler);

	/* 9. Configuramos para que el control del pin SS (selección del slave
	 *    sea controlado por software (nosotros debemos hacer ese control),
	 *    de la otra forma, será el hardware el que controla la selección del slave */
	pSpiHandler.pSPIx->CR1 |= SPI_CR1_SSM;
	pSpiHandler.pSPIx->CR1 |= SPI_CR1_SSI;

	/**/
	spi_config_interrupt(pSpiHandler);

	/* 10. Activamos el periférico SPI */
	pSpiHandler.pSPIx->CR1 |= SPI_CR1_SPE;

}

/*
 * Activa el la señal de reloj (RCC) para los SPI.
 */
static void spi_enable_clock_peripheral(SPI_Handler_t pSpiHandler){
	// Verificamos que periferico se esta utilizando y activa su
	// reloj adecuadamente.
	if(pSpiHandler.pSPIx == SPI1){
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	}
	else if(pSpiHandler.pSPIx == SPI2){
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
	else if(pSpiHandler.pSPIx == SPI3){
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	}
	else if(pSpiHandler.pSPIx == SPI4){
		RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
	}
	else if(pSpiHandler.pSPIx == SPI5){
		RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;
	}
}


/*
 * Configuracion del prescaler (baud) que maneja la velocidad con la que
 * se comunica el master con el esclavo
 */
static void spi_set_comm_speed(SPI_Handler_t pSpiHandler){
	// Limpiamos la posicion que deseamos configurar
	pSpiHandler.pSPIx->CR1 &= (0b11 << SPI_CR1_BR_Pos);

	// Cargamos el valor deseado en la posicion adecuada
	pSpiHandler.pSPIx->CR1 |= (pSpiHandler.SPI_Config.baudrate << SPI_CR1_BR_Pos);
}

/*
 * Configura uno de los cuatro modos en los que puede funcionar el SPI
 * Modo 00 -> muy tipico
 * Modo 01
 * Modo 10
 * Modo 11 -> muy tipico
 */
static void spi_set_mode(SPI_Handler_t pSpiHandler){

	// Limpiamos las posiciones que vamos a configurar
	pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPHA);
	pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPOL);

	// Cargamos el valor que deseamos configurar, por defecto es el modo 00
	switch(pSpiHandler.SPI_Config.mode){

	case SPI_MODE_00:
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPHA);
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPOL);
		break;

	case SPI_MODE_01:
		pSpiHandler.pSPIx->CR1 |=  (SPI_CR1_CPHA);
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPOL);
		break;

	case SPI_MODE_10:
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPHA);
		pSpiHandler.pSPIx->CR1 |=  (SPI_CR1_CPOL);
		break;

	case SPI_MODE_11:
		pSpiHandler.pSPIx->CR1 |= (SPI_CR1_CPHA);
		pSpiHandler.pSPIx->CR1 |= (SPI_CR1_CPOL);
		break;

	default:
		// Configuramos el mode_00 como caso por defecto
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPHA);
		pSpiHandler.pSPIx->CR1 &= ~(SPI_CR1_CPOL);
		break;
	}
}

/*
 * Se encarga de definir la direccion en la que se envia el dato
 * si MSB-first o si LSB-First
 */
static void spi_set_direction(SPI_Handler_t pSpiHandler){
	// Verificamos cual es la direccion deseada
	if(pSpiHandler.SPI_Config.direction == SPI_DIRECTION_MSBFIRST){
		pSpiHandler.pSPIx->CR1 &= ~SPI_CR1_LSBFIRST;
	}else{
		pSpiHandler.pSPIx->CR1 |= SPI_CR1_LSBFIRST;
	}

}

/*
 * Esta funcion se encarga de configurar si la comunicacion será
 * full-duplex o half-duplex
 */
static void spi_set_comm_type(SPI_Handler_t pSpiHandler){
	if(pSpiHandler.SPI_Config.fullDupplexEnable == SPI_FULL_DUPPLEX){
		// Selecciona full-dupplex
		pSpiHandler.pSPIx->CR1 &= ~SPI_CR1_RXONLY;
		// Selecciona modo uni-direccional (cada línea solo tiene una dirección)
		pSpiHandler.pSPIx->CR1 &= ~SPI_CR1_BIDIMODE;
	}
	else{
		// Selecciona solo RX activado
		pSpiHandler.pSPIx->CR1 |= SPI_CR1_RXONLY;
	}
}

/*
 * Configura si el tipo de dato a ser manejado es de 8 o 16 bit
 */
static void spi_set_data_size(SPI_Handler_t pSpiHandler){
	if(pSpiHandler.SPI_Config.datasize == SPI_DATASIZE_8_BIT){
		// framde de datos de 8-bit
		pSpiHandler.pSPIx->CR1 &= ~SPI_CR1_DFF;
	}else{
		// framde de datos de 16-bit
		pSpiHandler.pSPIx->CR1 |= SPI_CR1_DFF;
	}
}

/*
 * Configuracion de las posibles interrupiones que lanza el SPI
 * TX y RX
 */
static void spi_config_interrupt(SPI_Handler_t pSpiHandler){

	// Primero configuramos la parte individual, que son los activadores
	// en el registro de configuracion
	if(pSpiHandler.SPI_Config.txInterrupState == SPI_RX_INTERRUP_ENABLE){

		pSpiHandler.pSPIx->CR2 |= (SPI_CR2_RXNEIE);
	}else{
		pSpiHandler.pSPIx->CR2 &= ~(SPI_CR2_RXNEIE);
	}

	// Configuracion para la interrupcion TX
	if(pSpiHandler.SPI_Config.txInterrupState == SPI_TX_INTERRUP_ENABLE){
		pSpiHandler.pSPIx->CR2 |= (SPI_CR2_TXEIE);
	}else{
		pSpiHandler.pSPIx->CR2 &= ~(SPI_CR2_TXEIE);
	}

	// Y luego registramos la activacion de la interrupcione en el NVIC
	if((pSpiHandler.SPI_Config.rxInterrupState == SPI_RX_INTERRUP_ENABLE) ||
			(pSpiHandler.SPI_Config.rxInterrupState == SPI_TX_INTERRUP_ENABLE)){

		/* Activamos el canal del sistema NVIC para que lea la interrupción*/
		if(pSpiHandler.pSPIx == SPI1){
			NVIC_EnableIRQ(SPI1_IRQn);

		}
		else if(pSpiHandler.pSPIx == SPI2){
			NVIC_EnableIRQ(SPI2_IRQn);

		}else if(pSpiHandler.pSPIx == SPI3){
			NVIC_EnableIRQ(SPI3_IRQn);

		}else if(pSpiHandler.pSPIx == SPI4){
			NVIC_EnableIRQ(SPI4_IRQn);

		}else if(pSpiHandler.pSPIx == SPI5){
			NVIC_EnableIRQ(SPI5_IRQn);

		}

	}else{
		/* Desactivamos el canal del sistema NVIC del periferico */
		if(pSpiHandler.pSPIx == SPI1){
			NVIC_DisableIRQ(SPI1_IRQn);

		}
		else if(pSpiHandler.pSPIx == SPI2){
			NVIC_DisableIRQ(SPI2_IRQn);

		}else if(pSpiHandler.pSPIx == SPI3){
			NVIC_DisableIRQ(SPI3_IRQn);

		}else if(pSpiHandler.pSPIx == SPI4){
			NVIC_DisableIRQ(SPI4_IRQn);

		}else if(pSpiHandler.pSPIx == SPI5){
			NVIC_DisableIRQ(SPI5_IRQn);

		}
	}


}
/**/
void spi_Transmit(SPI_Handler_t pSpiHandler, uint8_t * ptrData, uint32_t dataSize){

	uint8_t auxData;
	(void) auxData;

	while(dataSize > 0){

		// Esperamos a que el buffer esté vacío
		while(!(pSpiHandler.pSPIx->SR & SPI_SR_TXE)){
			__NOP();
		}

		// Enviamos el dato al que apunta el puntero
		pSpiHandler.pSPIx->DR = (uint8_t) (0xFF & *ptrData);

		// Actualizamos el puntero y el número de datos que faltan por enviar
		ptrData++;
		dataSize--;
	}

	// Esperamos de nuevo a que el buffer esté vacío
	while (!(pSpiHandler.pSPIx->SR & SPI_SR_TXE)) {
		__NOP();
	}

	// Esperamos a que la bandera de ocupado (busy) baje (observar que la lógica cambia)
	while ((pSpiHandler.pSPIx->SR & SPI_SR_BSY)) {
		__NOP();
	}

	/* Debemos limpiar la bandera de OverRun (que a veces se levanta).
	 * Para esto debemos leer el DR y luego leer el SR del módulo SPI (pag 599) */
	auxData = pSpiHandler.pSPIx->DR;
	(void)auxData;
	auxData = pSpiHandler.pSPIx->SR;
	(void)auxData;
}


/**/
void spi_Receive(SPI_Handler_t pSpiHandler, uint8_t * ptrData, uint32_t dataSize){

	while(dataSize){
		// Esperamos de nuevo a que el buffer esté vacío
		while (!(pSpiHandler.pSPIx->SR & SPI_SR_TXE)) {
			__NOP();
		}
		// Enviamos un valor dummy
		pSpiHandler.pSPIx->DR = 0x00;

		// Esperamos de nuevo a que el buffer tenga un dato que leer
		while (!(pSpiHandler.pSPIx->SR & SPI_SR_RXNE)) {
			__NOP();
		}

		// Cargamos el valor en el puntero
		*ptrData = pSpiHandler.pSPIx->DR;

		// Actualizamos el puntero y el tamaño de los datos
		ptrData++;
		dataSize--;

	}
}

/* Seleccionamos el esclavo llevando el pin SS a GND */
void spi_SelectSlave(SPI_Handler_t* pSpiHandler){
	gpio_WritePin(&pSpiHandler->SPI_slavePin, RESET);
	//GPIOA->BSRR |= (SET << 25);
}

/* Seleccionamos el esclavo llevando el pin SS a Vcc */
void spi_UnselectSlave(SPI_Handler_t* pSpiHandler){
	gpio_WritePin(&pSpiHandler->SPI_slavePin, SET);
	//GPIOA->BSRR |= (SET << 9);
}




