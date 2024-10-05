/*
 * sensor.h
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <ringbufferUART.h>
#include "main.h"
#include "cmsis_os.h"

typedef struct {
    SPI_HandleTypeDef  * const hspi;
    UART_HandleTypeDef * const huart;
    RingBufferUART     * const uart_rb;
    osSemaphoreId_t    * const uartSemaphore;
    osMutexId_t        * const uartMutex;
} SENSOR_ARGS;

#endif /* INC_SENSOR_H_ */
