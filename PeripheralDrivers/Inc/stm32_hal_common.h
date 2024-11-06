/*
 * stm32_hal_common.h
 *
 *  Created on: Oct 30, 2024
 *      Author: namontoy
 */

#ifndef STM32_HAL_COMMON_H_
#define STM32_HAL_COMMON_H_

/**
  * @brief  HAL Status structures definition
  */
typedef enum
{
  HAL_CONFIG_OK,
  HAL_CONFIG_ERROR,
  HAL_GPIO_IS_CONFIGURE,
  HAL_TIMER_IS_CONFIGURE,
  HAL_USART_IS_CONFIGURE,
  HAL_SUCCESS
} eHAL_StatusMsg_t;

#endif /* STM32_HAL_COMMON_H_ */
