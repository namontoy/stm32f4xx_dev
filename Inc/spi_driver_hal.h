/*
 * spi_driver_hal.h
 *
 *  Created on: xx-xx, 2022
 *      Author: namontoy
 */

#ifndef SPI_DRIVER_HAL_H_
#define SPI_DRIVER_HAL_H_

#include "stm32f4xx.h"
#include "stdint.h"
#include "gpio_driver_hal.h"

enum {
	SPI_MODE_0 = 0,
	SPI_MODE_1,
	SPI_MODE_2,
	SPI_MODE_3
};

enum {
	SPI_BAUDRATE_FPCLK_2 = 0,
	SPI_BAUDRATE_FPCLK_4,
	SPI_BAUDRATE_FPCLK_8,
	SPI_BAUDRATE_FPCLK_16,
	SPI_BAUDRATE_FPCLK_32,
	SPI_BAUDRATE_FPCLK_64,
	SPI_BAUDRATE_FPCLK_128,
	SPI_BAUDRATE_FPCLK_256
};

enum{
	SPI_RECEIVE_ONLY = 0,
	SPI_FULL_DUPPLEX
};

enum{
	SPI_DIRECTION_MSBFIRST = 0,
	SPI_DIRECTION_LSBFIRST
};

enum{
	SPI_DATASIZE_8_BIT = 0,
	SPI_DATASIZE_16_BIT
};

enum
{
	SPI_RX_INTERRUP_DISABLE = 0,
	SPI_RX_INTERRUP_ENABLE,
	SPI_TX_INTERRUP_DISABLE,
	SPI_TX_INTERRUP_ENABLE
};


/**/
typedef struct
{
	uint8_t mode;				// Define los 4 diferentes modos que puede configurar el SPI.
	uint8_t baudrate;			// Define la velocidad que maneja el SPI con respecto al PCLK (Peripheral clock)
	uint8_t fullDupplexEnable;	// Configura si solo recepción o bi-direccional
	uint8_t	datasize;
	uint8_t	direction;
	uint8_t	txInterrupState;
	uint8_t	rxInterrupState;
}SPI_Config_t;

/**/
typedef struct
{
	SPI_TypeDef		*pSPIx;
	SPI_Config_t	SPI_Config;
	GPIO_Handler_t	SPI_slavePin;
}SPI_Handler_t;

/* Prototipos de las funciones */
void spi_Config(SPI_Handler_t ptrHandlerSPI);
void spi_Transmit(SPI_Handler_t ptrHandlerSPI, uint8_t * ptrData, uint32_t dataSize);
void spi_Receive(SPI_Handler_t ptrHandlerSPI, uint8_t * ptrData, uint32_t dataSize);
void spi_SelectSlave(SPI_Handler_t* ptrHandlerSPI);
void spi_UnselectSlave(SPI_Handler_t* ptrHandlerSPI);

#endif /* SPI_DRIVER_HAL_H_ */
