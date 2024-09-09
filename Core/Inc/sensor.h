/*
 * sensor.h
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "main.h"
#include "cmsis_os.h"
#include "ringbuffer.h"

typedef struct {
	SPI_HandleTypeDef * const hspi;
	UART_HandleTypeDef * const huart;
	RingBuffer * const uart_rb;
	osSemaphoreId_t * const uartSemaphore;
} SENSOR_ARGS;

#endif /* INC_SENSOR_H_ */
