/*
 * adc_driver_hal.h
 *
 *  Created on: Oct 11, 2023
 *      Author: namontoy
 */

#ifndef ADC_DRIVER_HAL_H_
#define ADC_DRIVER_HAL_H_

#include <stdint.h>
#include "stm32f4xx.h"

enum{
	CHANNEL_0 = 0,
	CHANNEL_1,
	CHANNEL_2,
	CHANNEL_3,
	CHANNEL_4,
	CHANNEL_5,
	CHANNEL_6,
	CHANNEL_7,
	CHANNEL_8,
	CHANNEL_9,
	CHANNEL_10,
	CHANNEL_11,
	CHANNEL_12,
	CHANNEL_13,
	CHANNEL_14,
	CHANNEL_15
};

enum{
	RESOLUTION_12_BIT =	0,
	RESOLUTION_10_BIT,
	RESOLUTION_8_BIT,
	RESOLUTION_6_BIT
};

enum{
	ALIGNMENT_RIGHT = 0,
	ALIGNMENT_LEFT
};

enum{
	SCAN_OFF = 0,
	SCAN_ON
};

enum{
	ADC_OFF = 0,
	ADC_ON
};

enum{
	ADC_INT_DISABLE = 0,
	ADC_INT_ENABLE
};

enum{
	TRIGGER_AUTO = 0,
	TRIGGER_MANUAL,
	TRIGGER_EXT
};

enum{
	SAMPLING_PERIOD_3_CYCLES = 0b000,
	SAMPLING_PERIOD_15_CYCLES = 0b001,
	SAMPLING_PERIOD_28_CYCLES = 0b010,
	SAMPLING_PERIOD_56_CYCLES = 0b011,
	SAMPLING_PERIOD_84_CYCLES = 0b100,
	SAMPLING_PERIOD_112_CYCLES = 0b101,
	SAMPLING_PERIOD_144_CYCLES = 0b110,
	SAMPLING_PERIOD_480_CYCLES = 0b111
};

/* ADC Handler definition
 * This handler is used to configure a single ADC channel.
 * - Channels 			-> configures inside the driver the correct GPIO pin as ADC channel
 * - Resolution 		-> 6bit, 8bit, 10 bit, 12 bit are the possible options.
 * - Sampling Period 	-> Related to the SARs procedure.
 * - Data alignment 	-> left or right, depends on the app. Default is right
 * - Adc data 			-> holds the data
 * */
typedef struct
{
	uint8_t		channel; 			// Canal ADC que sra utilizado para la conversion ADC
	uint8_t		resolution;			// Precision con la que el ADC hace la adquisicion del dato
	uint16_t	samplingPeriod;		// Tiempo deseado para hacer la adquisicion del dato
	uint8_t		dataAlignment;		// Alineacion a la izquierda
	uint16_t	adcData;			// Dato de la conversión
	uint8_t		interrupState;		// Para configurar si se desea o no trabajar con la interrupcion
} ADC_Config_t ;

/* Header definitions for the "public functions" of adc_driver_hal */
void adc_ConfigSingleChannel (ADC_Config_t *adcConfig);
void adc_ConfigAnalogPin(uint8_t adcChannel);
void adc_CompleteCallback (void);
void adc_StartSingleConv(void);
void adc_ScanMode(uint8_t state);
void adc_StartContinuousConv(void);
void adc_StopContinuousConv(void);
void adc_peripheralOnOFF(uint8_t state);
uint16_t adc_GetValue(void);

/* Configuraciones avanzadas del ADC */
void adc_ConfigMultichannel (ADC_Config_t *adcConfig, uint8_t numeroDeCanales);
//void adc_ConfigTrigger(uint8_t sourceType, PWM_Handler_t *triggerSignal);


#endif /* ADC_DRIVER_HAL_H_ */
