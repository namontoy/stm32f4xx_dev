/*
 * I2CDriver.c
 *
 *  Created on: XXXX, 2022
 *      Author: namontoy
 */

#include <stdint.h>
#include "i2c_driver_hal.h"
#include "gpio_driver_hal.h"


//GPIO_Handler_t	*sdaPin;
//GPIO_Handler_t	*sclPin;

/* === Headers for private functions === */
static void i2c_enable_clock_peripheral(I2C_Handler_t *pHandlerI2C);
static void i2c_soft_reset(I2C_Handler_t *pHandlerI2C);
static void i2c_set_main_clock(I2C_Handler_t *pHandlerI2C);
static void i2c_set_mode(I2C_Handler_t *pHandlerI2C);
static void i2c_enable_port(I2C_Handler_t *pHandlerI2C);
static void i2c_disable_port(I2C_Handler_t *pHandlerI2C);
static void i2c_stop_signal(I2C_Handler_t *pHandlerI2C);
static void i2c_start_signal(I2C_Handler_t *pHandlerI2C);
static void i2c_restart_signal(I2C_Handler_t *pHandlerI2C);
static void i2c_send_no_ack(I2C_Handler_t *ptrHandlerI2C);
static void i2c_send_ack(I2C_Handler_t *pHandlerI2C);
static void i2c_send_slave_address_rw(I2C_Handler_t *pHandlerI2C, uint8_t rw);
static void i2c_send_memory_address(I2C_Handler_t *pHandlerI2C, uint8_t memAddr);
static void i2c_send_close_comm(I2C_Handler_t *pHandlerI2C);
static void i2c_send_byte(I2C_Handler_t *pHandlerI2C, uint8_t dataToWrite);
static uint8_t i2c_read_byte(I2C_Handler_t *pHandlerI2C);

//static void i2c_config_interrupt(I2C_Handler_t *pHandlerI2C);

/*
 * Recordar que se debe configurar los pines para el I2C (SDA y SCL),
 * para lo cual se necesita el modulo GPIO y los pines configurados
 * en el modo Alternate Function.
 * Además, estos pines deben ser configurados como salidas open-drain
 * y con la resistencias en modo pull-up.
 */
void i2c_Config(I2C_Handler_t *pHandlerI2C)
{
	/* 1. Activamos la señal de reloj para el periferico */
	i2c_enable_clock_peripheral(pHandlerI2C);

	/* disable i2c port */
	i2c_disable_port(pHandlerI2C);

	/* 2. Reiniciamos el periférico, de forma que inicia en un estado conocido */
	i2c_soft_reset(pHandlerI2C);

	/* 3. Indicamos cual es la velocidad del reloj principal, que es la señal utilizada
	 * por el periférico para generar la señal de reloj para el bus I2C*/
	i2c_set_main_clock(pHandlerI2C);

	/* 4. Configuramos el modo I2C en el que el sistema funciona
	 * En esta configuración se incluye también la velocidad del reloj
	 * y el tiempo máximo para el cambio de la señal (T-Rise). */
	i2c_set_mode(pHandlerI2C);

	/* 5. Activamos el modulo I2C */
	i2c_enable_port(pHandlerI2C);

	//i2c_stopTransaction(ptrHandlerI2C);

}

/*
 * Activa el la señal de reloj (RCC) para los I2C.
 * */
static void i2c_enable_clock_peripheral(I2C_Handler_t *pHandlerI2C)
{
	/* 1 Activamos la señal de reloj para el módulo I2C seleccionado */
	if(pHandlerI2C->pI2Cx == I2C1){
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}

	else if(pHandlerI2C->pI2Cx == I2C2){
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}

	else if(pHandlerI2C->pI2Cx == I2C3){
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	}
}


/**/
static void i2c_set_main_clock(I2C_Handler_t *pHandlerI2C)
{
	pHandlerI2C->pI2Cx->CR2 &= ~(0b111111 << I2C_CR2_FREQ_Pos);  // Borramos la configuración previa
	pHandlerI2C->pI2Cx->CR2 |= (pHandlerI2C->i2c_mainClock << I2C_CR2_FREQ_Pos);
}

/**/
static void i2c_set_mode(I2C_Handler_t *pHandlerI2C)
{
	/* Borramos la informacion de ambos registros (aunque esto lo debe hacer el reset) */
	pHandlerI2C->pI2Cx->CCR = 0;
	pHandlerI2C->pI2Cx->TRISE = 0;

	if(pHandlerI2C->i2c_mode == eI2C_MODE_SM){

		//Estamos en modo "standar" (SM Mode)
		// Seleccionamos el modo estandar
		pHandlerI2C->pI2Cx->CCR &= ~I2C_CCR_FS;

		// Configuramos el registro que se encarga de generar la señal del reloj
		pHandlerI2C->pI2Cx->CCR |= (I2C_MODE_SM_SPEED << I2C_CCR_CCR_Pos);

		// Configuramos el registro que controla el tiempo T-Rise máximo
		pHandlerI2C->pI2Cx->TRISE |= I2C_MAX_RISE_TIME_SM;

	}
	else{
		// Estamos en modo "Fast" (FM mode)
		// Seleccionamos el modo Fast
		pHandlerI2C->pI2Cx->CCR |= I2C_CCR_FS;

		// Configuramos el registro que se encarga de generar la señal del reloj
		pHandlerI2C->pI2Cx->CCR |= (I2C_MODE_FM_SPEED << I2C_CCR_CCR_Pos);

		// Configuramos el registro que controla el tiempo T-Rise máximo
		pHandlerI2C->pI2Cx->TRISE |= I2C_MAX_RISE_TIME_FM;
	}
}

/*
 * Activa el puerto, sin esto el sistema no funciona.
 * El puerto debe estar desactivado para poder ser configurado, si se intenta
 * configurar cuando esta activo se genera un error
 * */
static void i2c_enable_port(I2C_Handler_t *pHandlerI2C)
{
	pHandlerI2C->pI2Cx->CR1 |= I2C_CR1_PE;
}

/*
 * Desactiva el puerto, haciendo que el sistema no fucniona y habilita para que se pueda configurar.
 * El puerto debe estar desactivado para poder ser configurado, si se intenta
 * configurar cuando esta activo se genera un error
 * */
static void i2c_disable_port(I2C_Handler_t *pHandlerI2C)
{
	pHandlerI2C->pI2Cx->CR1 &= ~I2C_CR1_PE;
}

/*
 * Esta funcion se encarga de llevar a 0 todos los bits relacionados con el periferico I2C
 * que se esta utilizando
 * */
static void i2c_soft_reset(I2C_Handler_t *pHandlerI2C)
{
	pHandlerI2C->pI2Cx->CR1 |= I2C_CR1_SWRST;
	__NOP();
	__NOP();
	pHandlerI2C->pI2Cx->CR1 &= ~I2C_CR1_SWRST;
}

/* 8. Generamos la condición de stop */
static void i2c_stop_signal(I2C_Handler_t *pHandlerI2C)
{
	/* 7. Generamos la condición de stop */
	pHandlerI2C->pI2Cx->CR1 |= I2C_CR1_STOP;
}

/*
 * Funcion que genera la señal START de un ciclo de comunicación del I2C.
 * El código está relacionado con la figura 164 (Pag 481) del manual de referencia
 * del MCU.
 *
 * 1. Configuramos bit I2C_CR1_POS
 * 2. Generamos la señal "start"
 * 2a. Esperamos a que la bandera del evento "start" se levante.
 * (mientras esperamos, el valor de SB es 0, entonces la negación (!) es 1)
 * 3. Leemos el registro SR1
 * Estos pasos hacen parte del evento EV5 de la figura 164.
 * */
static void i2c_start_signal(I2C_Handler_t *pHandlerI2C)
{
	/* 0. Definimos una variable auxiliar */
	uint8_t auxByte = 0;
	(void) auxByte;

	// Errata del MCU... solución en el foro de ST.
	// usuario "ERol.1"
	// https://community.st.com/t5/stm32-mcus-products/stm32f2xx-i2c-not-sending-address-after-start/td-p/423510
	//ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_STOP;

	/* 0. Reset, para tener el periferico en un estado conocido. */
	//i2c_softReset(ptrHandlerI2C);

	/* 1. Configuramos el control para generar el bit ACK.
	 * Este bit posición lo que hace es controlar si nuestro bit ACK genera dicha
	 * señal para el byte que se está leyendo actualmente (I2C_CR1_POS = 0) o para el byte
	 * que llegará posteriormente (I2C_CR1_POS = 1).
	 * Lo mas lógico es trabajar con el byte que se está recibiendo actualmente.
	 * */
	pHandlerI2C->pI2Cx->CR1 &= ~I2C_CR1_POS;

	/* 2. Generamos la señal "start" */
	pHandlerI2C->pI2Cx->CR1 |= I2C_CR1_START;

	/* 2a. Esperamos a que la bandera del evento "start" se levante.
	 * Este bit se hace 1 si y solo si una señal de star se genera satisfactoriamente.
	 * Mientras esperamos, el valor de SB es 0, entonces la negación <(!) es 1
	 * */
	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_SB)){
		__NOP();
	}

	/* El sistema espera que el registro SR1 sea leido, para continuar con el siguiente paso, que es
	 * enviar la señal del esclavo (siempre)
	 * Pag 479 del manual
	 * Esta condición de leer el registro SR1 es necesaria y siempre se debe hacer.
	 * */
	auxByte = pHandlerI2C->pI2Cx->SR1;
	(void)auxByte;

}

/* Segun el manual, hacer una señal de start luego de la transferencia de un byte
 * nos genera una señal de restart, por lo cual es suficiente con hacer un nuevo start
 * */
static void i2c_restart_signal(I2C_Handler_t *pHandlerI2C)
{
	/* 2. Generamos la señal "start" */
	i2c_start_signal(pHandlerI2C);
}

/*
 * Activamos la indicación para no-ACK, la cual es utilizada en el protocolo I2C
 * como indicación para el Slave de terminar de transmitir un conjunto de datos
 * */
static void i2c_send_no_ack(I2C_Handler_t *pHandlerI2C)
{
	 /* (Debemos escribir cero en la posición ACK del registro de control 1) */
	pHandlerI2C->pI2Cx->CR1 &= ~I2C_CR1_ACK;
}

/*
 * Activamos la indicación para ACK, la cual es utilizada por el protocolo I2C
 * como indicacion para el Slave, para que continue enviando información, o sea
 * para que envie el siguiente byte en memoria
 *  */
static void i2c_send_ack(I2C_Handler_t *pHandlerI2C)
{
	/* Escribimos 'uno' en la posición ACK del registro de control 1 */
	pHandlerI2C->pI2Cx->CR1 |= I2C_CR1_ACK;
}

/*
 * Este es el paso siguente a la señal start (o restart) y siempre es la dirección del
 * esclavo mas la indicacion de si se desea leer (1) o escribir (0)
 * Se debe tener en cuenta que la dirección del esclavo será movida una casilla de derecha a izquierda
 * para generar el "espacio" para la indicacion R/W, por lo cual la dirección del esclado tiene
 * que ser escrita correctamente desde el inicio, si esto no se hace el sistema enviará una dirección
 * que simplemente el esclavo no va a entender.
 * esto es, se hace la operación: (slaveAddress << 1) | R/W
 *
 * Descrito entre los eventos EV6 y EV6 de la figura 164.
 * */
static void i2c_send_slave_address_rw(I2C_Handler_t *pHandlerI2C, uint8_t rw)
{
	/* 0. Definimos una variable auxiliar */
	uint8_t auxByte = 0;
	(void) auxByte;

	/* 3. Enviamos la dirección del Slave y el bit que indica que deseamos escribir (0)
	 * (en el siguiente paso se envia la dirección de memoria que se desea escribir */
	pHandlerI2C->pI2Cx->DR = (pHandlerI2C->slaveAddress << 1) | rw;

	/* 3.1. Esperamos hasta que la bandera del evento "addr" se levante
	 * (esto nos indica que la dirección fue enviada satisfactoriamente*/
	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_ADDR)){
		__NOP();
	}

	/* 3.2 Debemos limpiar la bandera de la recepción de ACK de la "addr", para lo cual
	 * debemos leer en secuencia primero el I2C_SR1 y luego I2C_SR2 */
	auxByte = pHandlerI2C->pI2Cx->SR1;
	(void)auxByte;
	auxByte = pHandlerI2C->pI2Cx->SR2;
	(void)auxByte;
	/* El bit TRA del registro SR2 nos indica si el equiopo quedó en modo de transmision
	 * o en modo de recepcion, lo cual esta definido por el tipo de seleccion de R/W que se envio
	 * pag 480 del manual.
	 * */

	/* Esperamos hasta que el byte sea montado en el DSR, quedando el DR libre de nuevo
	 * Este paso no ews claramente especificado en el manual, pero es la forma de verificar
	 * que se puede continuar, ya qye el DR quedará libre.
	 * */
//	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_TXE)){
//		__NOP();
//	}
	__NOP(); // para verificar en modo debug.
}

/**/
static void i2c_send_byte(I2C_Handler_t *pHandlerI2C, uint8_t dataToWrite)
{
	/* 5. Cargamos el valor que deseamos escribir */
	pHandlerI2C->pI2Cx->DR = dataToWrite;

	/* 5.1 Esperamos hasta que el byte sea montado en el DSR, quedando el DR libre de nuevo */
	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_TXE)){
		__NOP();
	}
}

/*
 * Esta funcion se aprovecha del hecho que enviar la dirección de la memoria que se desea leer
 * es igual a enviar un byte X cualquiera, por lo cual simplemente se llama a la funcion enviar un
 * byte generico, el cual es la posición de memoria que se desea leer
 * */
void i2c_send_memory_address(I2C_Handler_t *pHandlerI2C, uint8_t memAddr){
	i2c_send_byte(pHandlerI2C, memAddr);
}

/*
 * Esta funcion evalua que el ultimo byte ha sido transmitico completamente por el TSR,
 * luego de esto, transmite el señal de stop
 * */
void i2c_send_close_comm(I2C_Handler_t *pHandlerI2C)
{
	/* 5.1 Esperamos hasta que el ultimo byte sea transmitido completamente por el TSR
	 * lo cual activa el bit BTF, quedando tanto BTF como TXE en 1
	 * */
//	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_BTF)){
//		__NOP();
//	}

	/* Enviamos la señal de detencion */
	i2c_stop_signal(pHandlerI2C);

}

/**/
uint8_t i2c_read_byte(I2C_Handler_t *pHandlerI2C)
{
	/* 9. Esperamos hasta que el byte entrante sea recibido */
	while( !(pHandlerI2C->pI2Cx->SR1 & I2C_SR1_RXNE)){
		__NOP();
	}

	pHandlerI2C->i2c_data = pHandlerI2C->pI2Cx->DR;

	return pHandlerI2C->i2c_data;
}

/* ===== Funciones Publicas del Driver ============*/

/**/
uint8_t i2c_ReadSingleRegister(I2C_Handler_t *pHandlerI2C, uint8_t regToRead){

	/* 0. Creamos una variales auxiliar para recibir el dato que leemos*/
	uint8_t auxRead = 0;

	/* 1. Generamos la condición Start */
	i2c_start_signal(pHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos leer */
	i2c_send_memory_address(pHandlerI2C, regToRead);

	/* 4. Creamos una condición re reStart */
	i2c_restart_signal(pHandlerI2C);

	/* 5. Enviamos la dirección del esclavo y la indicación de LEER */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_READ_DATA);

	/* 8. Leemos el dato que envia el esclavo */
	auxRead = i2c_read_byte(pHandlerI2C);

	/* 6. Generamos la condición de NoAck, para que el Master no responda y el slave solo envie 1 byte*/
	i2c_send_no_ack(pHandlerI2C);

	/* 7. Generamos la condición Stop, para que el slave se detenga despues de 1 byte */
	i2c_stop_signal(pHandlerI2C);
	//i2c_send_close_comm(pHandlerI2C);


	return auxRead;
}

/**/
uint8_t i2c_ReadManyRegisters(I2C_Handler_t *pHandlerI2C, uint8_t regToRead, uint8_t *bufferRxData, uint8_t numberOfBytes){

	/* 1. Generamos la condición Start */
	i2c_start_signal(pHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria desde donde deseamos leer */
	i2c_send_memory_address(pHandlerI2C, regToRead);

	/* 4. Creamos una condición re reStart */
	i2c_restart_signal(pHandlerI2C);

	/* 5. Enviamos la dirección del esclavo y la indicación de LEER */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_READ_DATA);

	/* Activamos el envio de ACK */
	i2c_send_ack(pHandlerI2C);

	/* 8. Leemos el dato que envia el esclavo */
	while(numberOfBytes > 0){
		/* Cuando numberOfBytes es igual a 1, significa que es el ultimo y hay que parar */
		if(numberOfBytes == 1){
			/* 6. Generamos la condición de NoAck, para que el Master no responda y el slave solo envie 1 byte*/
			i2c_send_no_ack(pHandlerI2C);

			/* 7. Generamos la condición Stop, para que el slave se detenga despues de 1 byte */
			i2c_send_close_comm(pHandlerI2C);

			*bufferRxData = i2c_read_byte(pHandlerI2C);
		}
		/* Si no es el ultimo, entonces captura el valor, incrementa el puntero y reduce en 1 */
		else{
			*bufferRxData = i2c_read_byte(pHandlerI2C);
			bufferRxData++;
		}
		/* Reducimos en 1 el conteo de los bytes a recibir */
		numberOfBytes--;
	}

	return numberOfBytes;
}

/**/
void i2c_WriteSingleRegister(I2C_Handler_t *pHandlerI2C, uint8_t regToWrite, uint8_t newValue){

	/* 1. Generamos la condición Start */
	i2c_start_signal(pHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos escribir */
	i2c_send_memory_address(pHandlerI2C, regToWrite);

	/* 4. Enviamos el valor que deseamos escribir en el registro seleccionado */
	i2c_send_byte(pHandlerI2C, newValue);

	/* 5. Generamos la condición Stop, para que el slave se detenga despues de 1 byte */
	i2c_send_close_comm(pHandlerI2C);
}

/**/
void i2c_WriteManyRegisters(I2C_Handler_t *pHandlerI2C, uint8_t regToWrite, uint8_t *bufferRxData, uint8_t numberOfBytes){

	/* 1. Generamos la condición Start */
	i2c_start_signal(pHandlerI2C);

	/* 2. Enviamos la dirección del esclavo y la indicación de ESCRIBIR */
	i2c_send_slave_address_rw(pHandlerI2C, eI2C_WRITE_DATA);

	/* 3. Enviamos la dirección de memoria que deseamos escribir */
	i2c_send_memory_address(pHandlerI2C, regToWrite);

	while(numberOfBytes > 0){
		/* 4. Enviamos el valor que deseamos escribir en el registro seleccionado */
		i2c_send_byte(pHandlerI2C, *bufferRxData);
		bufferRxData++;
		numberOfBytes--;
	}

	/* 5. Generamos la condición Stop, para que el slave se detenga despues de 1 byte */
	i2c_send_close_comm(pHandlerI2C);
}

///**/
//void i2c_setPins(GPIO_Handler_t	*setSdaPin, GPIO_Handler_t	*setSclPin){
//	sdaPin = setSdaPin;
//	sclPin = setSclPin;
//}
//
///*
// * Siguiendo la errata para cambiar el estado de los filtros analogos
// * pagina 23.
// * */
//void i2c_clearBusyFlagState(I2C_Handler_t *ptrHandlerI2C){
//	/* 1. Disable the peripheral*/
//	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_PE;
//
//	ptrHandlerI2C->ptrI2Cx->FLTR &= ~I2C_FLTR_ANOFF;
//
//	/* 2a. Config como general purpose, open-drain, set high state */
//	sclPin->pinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	sclPin->pinConfig.GPIO_PinOutputType = GPIO_OTYPE_OPENDRAIN;
//	sclPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	sclPin->pinConfig.GPIO_PinOutputSpeed = GPIO_OSPEED_FAST;
//	gpio_Config(sclPin);
//	gpio_WritePin(sclPin, 1);
//
//	sdaPin->pinConfig.GPIO_PinMode		= GPIO_MODE_OUT;
//	sdaPin->pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_OPENDRAIN;
//	sdaPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	sdaPin->pinConfig.GPIO_PinOutputSpeed = GPIO_OSPEED_FAST;
//	gpio_Config(sdaPin);
//	gpio_WritePin(sdaPin, 1);
//
//
////	sclPin->pinConfig.GPIO_PinMode = GPIO_MODE_IN;
////	sclPin->pinConfig.GPIO_PinOutputType = GPIO_OTYPE_OPENDRAIN;
////	sclPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
////	sclPin->pinConfig.GPIO_PinOutputSpeed = GPIO_OSPEED_FAST;
////	gpio_Config(sclPin);
////
////	sdaPin->pinConfig.GPIO_PinMode		= GPIO_MODE_IN;
////	sdaPin->pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_OPENDRAIN;
////	sdaPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
////	sdaPin->pinConfig.GPIO_PinOutputSpeed = GPIO_OSPEED_FAST;
////	gpio_Config(sdaPin);
//
//	/*3. Verificar el estado de los pines, que si van a 1 */
//	while(gpio_ReadPin(sdaPin) != 1){
//		__NOP();
//	}
//
//	while(gpio_ReadPin(sclPin) != 1){
//		__NOP();
//	}
//
//	/* 4. Cambiar el estado a bajo */
//	gpio_WritePin(sdaPin, 0);
//
//	/* 5. Verificar que estan en bajo */
//	while(gpio_ReadPin(sdaPin) != 0){
//		__NOP();
//	}
//
//	/* 6. Cambiar el estado a bajo el scl */
//	gpio_WritePin(sclPin, 0);
//
//	/* 7. Verificar el estado del scl */
//	while(gpio_ReadPin(sclPin) != 0){
//		__NOP();
//	}
//
//	/* 8. Cambiar el estado a bajo */
//	gpio_WritePin(sclPin, 1);
//
//	/* 9. Verificar que estan en alto */
//	while(gpio_ReadPin(sclPin) != 1){
//		__NOP();
//	}
//
//	/* 10. Cambiar el estado a bajo el sda */
//	gpio_WritePin(sdaPin, 1);
//
//	/* 11. Verificar el estado del sda */
//	while(gpio_ReadPin(sdaPin) != 1){
//		__NOP();
//	}
//
//	/* 12. Config como general purpose, open-drain, set high state */
//	sdaPin->pinConfig.GPIO_PinMode		= GPIO_MODE_ALTFN;
//	sdaPin->pinConfig.GPIO_PinOutputType	= GPIO_OTYPE_OPENDRAIN;
//	sdaPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	sdaPin->pinConfig.GPIO_PinAltFunMode = AF4;
//	gpio_Config(sdaPin);
//
//	sclPin->pinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//	sclPin->pinConfig.GPIO_PinOutputType = GPIO_OTYPE_OPENDRAIN;
//	sclPin->pinConfig.GPIO_PinPuPdControl = GPIO_PUPDR_NOTHING;
//	sclPin->pinConfig.GPIO_PinAltFunMode = AF4;
//	gpio_Config(sclPin);
//
//	/* pasos 13 y 14, llegar a 1 y luego a 0 el bit SWRST del CR2 */
//	//i2c_softReset(ptrHandlerI2C);
//
//	ptrHandlerI2C->ptrI2Cx->FLTR |= I2C_FLTR_ANOFF;
//
//	ptrHandlerI2C->ptrI2Cx->CR1 &= ~I2C_CR1_SWRST;
//
//	/* Activar de nuevo el periferico */
//	ptrHandlerI2C->ptrI2Cx->CR1 |= I2C_CR1_PE;
//
//	i2c_stopTransaction(ptrHandlerI2C);
//
//}

