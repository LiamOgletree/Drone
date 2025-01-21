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
    UART_HandleTypeDef * const huart_logger;
    UART_HandleTypeDef * const huart_pa1616s;
    RingBuffer         * const logger_ringbuffer;
    RingBuffer         * const sm_ringbuffer;
    RingBuffer         * const pa1616s_ringbuffer;
    osSemaphoreId_t    * const logger_semaphore;
    osSemaphoreId_t    * const sm_semaphore;
    osSemaphoreId_t    * const pa1616s_semaphore;
    osMutexId_t        * const logger_mutex;
    osMutexId_t        * const sm_mutex;
} TASK_ARGS;

void task_report_error(TASK_ARGS const args,
                       char * const message);
void task_send_update_all(TASK_ARGS const args,
                          RingBuffer_t const update);
void task_send_update_log(TASK_ARGS const args,
                          RingBuffer_t const update);

#endif /* INC_TASKSHARED_H_ */
