/*
 * debug_driver_hal.h
 *
 *  Created on: Nov 10, 2023
 *      Author: namontoy
 */

#ifndef DEBUG_DRIVER_HAL_H_
#define DEBUG_DRIVER_HAL_H_

enum{
	CYCCNT_STOP = 0,
	CYCCNT_START
};

void debug_Enable_Trace(void);
void debug_Disable_Trace(void);
uint8_t debug_Check_Trace_Available(void);
void debug_StartStop_CycleCounter(uint8_t state);


#endif /* DEBUG_DRIVER_HAL_H_ */
