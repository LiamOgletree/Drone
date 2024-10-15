/*
 * sensor.h
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#ifndef INC_TASKSHARED_H_
#define INC_TASKSHARED_H_

#define TIM5_CLK_SPEED (80000000.)

#include "RingBuffer.h"
#include "cmsis_os.h"
#include <stdbool.h>

typedef struct {
    SPI_HandleTypeDef  * const hspi;
    UART_HandleTypeDef * const huart;
    RingBuffer         * const uart_rb;
    RingBuffer         * const sm_rb;
    osSemaphoreId_t    * const uartSemaphore;
    osSemaphoreId_t    * const smSemaphore;
    osMutexId_t        * const uartMutex;
    osMutexId_t        * const smMutex;
} TASK_ARGS;

void task_report_error(TASK_ARGS const args,
                       char * const message);
void task_send_update_all(TASK_ARGS const args,
                          RingBuffer_t const update);
void task_send_update_log(TASK_ARGS const args,
                          RingBuffer_t const update);

#endif /* INC_TASKSHARED_H_ */
