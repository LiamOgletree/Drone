/*
 * shared.c
 *
 *  Created on: Oct 8, 2024
 *      Author: liamt
 */

#include <taskShared.h>

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void task_report_error(TASK_ARGS const args,
                       char * const message)
{
    // Create temporary RingBuffer_t object to place in UART RingBuffer.
    RingBuffer_t const tmp = {.type = RB_ERROR,
                              .error_buf = message};

    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.logger_mutex, osWaitForever);
    RingBuffer_enqueue(args.logger_ringbuffer, tmp);
    osMutexRelease(*args.logger_mutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.logger_semaphore);
}

void task_send_update_all(TASK_ARGS const args,
                          RingBuffer_t const update)
{
    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.sm_mutex, osWaitForever);
    RingBuffer_enqueue(args.sm_ringbuffer, update);
    osMutexRelease(*args.sm_mutex);

    // Notify the State Machine task that a RingBuffer_t object
    //  has been enqueued.
    osSemaphoreRelease(*args.sm_semaphore);

    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.logger_mutex, osWaitForever);
    RingBuffer_enqueue(args.logger_ringbuffer, update);
    osMutexRelease(*args.logger_mutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.logger_semaphore);
}

void task_send_update_log(TASK_ARGS const args,
                          RingBuffer_t const update)
{
    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.logger_mutex, osWaitForever);
    RingBuffer_enqueue(args.logger_ringbuffer, update);
    osMutexRelease(*args.logger_mutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.logger_semaphore);
}
