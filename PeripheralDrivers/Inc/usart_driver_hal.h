/*
 * usart_driver_hal.h
 *
 *  Created on: Oct 5, 2023
 *      Author: ingfisica
 */

#ifndef USART_DRIVER_HAL_H_
#define USART_DRIVER_HAL_H_

#include <stdio.h>
#include "stm32f4xx.h"


enum
{
	USART_MODE_TX = 0,
	USART_MODE_RX,
	USART_MODE_RXTX,
	USART_MODE_DISABLE
};

enum
{
	USART_RX_INTERRUP_DISABLE = 0,
	USART_RX_INTERRUP_ENABLE,
	USART_TX_INTERRUP_DISABLE,
	USART_TX_INTERRUP_ENABLE
};

enum{
	USART_BAUDRATE_9600 = 0,
	USART_BAUDRATE_19200,
	USART_BAUDRATE_115200,
	USART_BAUDRATE_230400,
	USART_BAUDRATE_921600
};

enum{
	USART_DATASIZE_8BIT = 0,
	USART_DATASIZE_9BIT
};

enum{
	USART_PARITY_NONE = 0,
	USART_PARITY_ODD,
	USART_PARITY_EVEN
};

enum{
	USART_STOPBIT_1 = 0,
	USART_STOPBIT_0_5,
	USART_STOPBIT_2,
	USART_STOPBIT_1_5
};

enum
{
	e_USART_PRIORITY_6	= 6,
	e_USART_PRIORITY_7,
	e_USART_PRIORITY_8,
	e_USART_PRIORITY_9,
	e_USART_PRIORITY_10,
	e_USART_PRIORITY_11,
	e_USART_PRIORITY_12,
	e_USART_PRIORITY_13,
	e_USART_PRIORITY_14,
	e_USART_PRIORITY_15
};


/* Estructura para la configuración de la comunicacion:
 * Velocidad (baudrate)
 * Tamaño de los datos
 * Control de errores
 * Bit de parada
 */
typedef struct
{
	uint8_t mode;
	uint8_t baudrate;
	uint8_t datasize;
	uint8_t parity;
	uint8_t stopbits;
	uint8_t	enableIntRX;
	uint8_t	enableIntTX;
}USART_Config_t;

/*
 * Definicion del Handler para un USART:
 * - Estructura que contiene los SFR que controlan el periferico
 * - Estructura que contiene la configuración especifica del objeto
 * - Buffer de recepcion de datos
 * - Elemento que indica cuantos datos se recibieron
 * - Buffer de transmision de datos
 * - Elemento que indica cuantos datos se deben enviar.
 */
typedef struct
{
	USART_TypeDef	*ptrUSARTx;
	USART_Config_t	USART_Config;
	uint8_t			receptionBuffer[64];
	uint8_t			dataInputSize;
	uint8_t			transmisionBuffer[64];
	uint8_t			dataOutputSize;
}USART_Handler_t;



/* Definicion de los prototipos para las funciones del USART */
void usart_Config(USART_Handler_t *ptrUsartHandler);
void usart_Config_Int_Priority(USART_Handler_t *ptrUsartHandler, uint8_t newPriority);
int  usart_WriteChar(USART_Handler_t *ptrUsartHandler, int dataToSend );
void usart_writeMsg(USART_Handler_t *ptrUsartHandler, char *msgToSend );
uint8_t usart_getRxData(void);
void usart1_RxCallback(void);
void usart2_RxCallback(void);
void usart6_RxCallback(void);


#endif /* USART_DRIVER_HAL_H_ */
