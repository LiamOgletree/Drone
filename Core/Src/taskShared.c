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
    osMutexAcquire(*args.uartMutex, osWaitForever);
    RingBuffer_enqueue(args.uart_rb, tmp);
    osMutexRelease(*args.uartMutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.uartSemaphore);
}

void task_send_update_all(TASK_ARGS const args,
                          RingBuffer_t const update)
{
    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.smMutex, osWaitForever);
    RingBuffer_enqueue(args.sm_rb, update);
    osMutexRelease(*args.smMutex);

    // Notify the State Machine task that a RingBuffer_t object
    //  has been enqueued.
    osSemaphoreRelease(*args.smSemaphore);

    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.uartMutex, osWaitForever);
    RingBuffer_enqueue(args.uart_rb, update);
    osMutexRelease(*args.uartMutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.uartSemaphore);
}

void task_send_update_log(TASK_ARGS const args,
                          RingBuffer_t const update)
{
    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.uartMutex, osWaitForever);
    RingBuffer_enqueue(args.uart_rb, update);
    osMutexRelease(*args.uartMutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.uartSemaphore);
}
