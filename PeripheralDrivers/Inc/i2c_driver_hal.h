/*
 * i2c_driver_hal.h
 *
 *  Created on: Dec 1, 2023
 *      Author: namontoy
 */

#ifndef I2C_DRIVER_HAL_H_
#define I2C_DRIVER_HAL_H_

#include <stm32f4xx.h>
#include "gpio_driver_hal.h"

#define I2C_WRITE_DATA		0
#define I2C_READ_DATA		1

#define MAIN_CLOCK_4_MHz_FOR_I2C	4
#define MAIN_CLOCK_16_MHz_FOR_I2C	16
#define MAIN_CLOCK_20_MHz_FOR_I2C	20

#define I2C_MODE_SM			0
#define I2C_MODE_FM			1

#define I2C_MODE_SM_SPEED_100KHz	80
#define I2C_MODE_FM_SPEED_400KHz	13

#define I2C_MAX_RISE_TIME_SM		17
#define I2C_MAX_RISE_TIME_FM		5



typedef struct
{
	I2C_TypeDef		*ptrI2Cx;
	uint8_t			slaveAddress;
	uint8_t			modeI2C;
	uint8_t			dataI2C;
}I2C_Handler_t;

/*Prototipos de las funciones públicas */
void i2c_config(I2C_Handler_t *ptrHandlerI2C);
void i2c_startTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_reStartTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendSlaveAddressRW(I2C_Handler_t *ptrHandlerI2C, uint8_t slaveAddress, uint8_t readOrWrite);
void i2c_sendMemoryAddress(I2C_Handler_t *ptrHandlerI2C, uint8_t memAddr);
void i2c_sendDataByte(I2C_Handler_t *ptrHandlerI2C, uint8_t dataToWrite);
uint8_t i2c_readDataByte(I2C_Handler_t *ptrHandlerI2C);
void i2c_stopTransaction(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendAck(I2C_Handler_t *ptrHandlerI2C);
void i2c_sendNoAck(I2C_Handler_t *ptrHandlerI2C);
void i2c_softReset(I2C_Handler_t *ptrHandlerI2C);

void i2c_setPins(GPIO_Handler_t	*setSdaPin, GPIO_Handler_t	*setSclPin);
void i2c_clearBusyFlagState(I2C_Handler_t *ptrHandlerI2C);

uint8_t i2c_readSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead);
uint8_t i2c_readManyRegisters(I2C_Handler_t *ptrHandlerI2C, uint8_t regToRead, uint8_t *bufferData, uint8_t numberOfBytes);
void i2c_writeSingleRegister(I2C_Handler_t *ptrHandlerI2C, uint8_t regToWrite, uint8_t newValue);
void i2c_writeManyRegisters(I2C_Handler_t *ptrHandlerI2C, uint8_t regToWrite, uint8_t *bufferRxData, uint8_t numberOfBytes);


#endif /* I2C_DRIVER_HAL_H_ */
