/*
 * taskLIS2MDL.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLIS2MDL.h"
#include "sensor.h"
#include "LIS2MDL.h"

// TODO:
//  1. Try to recover LIS2MDL functionality instead of suspending task.

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline void report_error(SENSOR_ARGS const args,
                                char * const message)
{
    // Create temporary RingBuffer_t object to place in UART RingBuffer.
    RingBufferUART_t const tmp = {.type = UPDATE_ERROR,
                                  .error_buf = message};

    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.uartMutex, osWaitForever);
    RingBufferUART_enqueue(args.uart_rb, tmp);
    osMutexRelease(*args.uartMutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.uartSemaphore);
}

static inline void send_update(SENSOR_ARGS const args,
                               LIS2MDL const lis2mdl)
{
    // Create temporary RingBuffer_t object to place in UART RingBuffer.
    RingBufferUART_t const tmp = {.type = UPDATE_LIS2MDL,
                                  .lis2mdl = lis2mdl};

    // Enqueue the RingBuffer_t object without creating race conditions
    //  with other tasks either enqueueing or dequeueing from the RingBuffer.
    osMutexAcquire(*args.uartMutex, osWaitForever);
    RingBufferUART_enqueue(args.uart_rb, tmp);
    osMutexRelease(*args.uartMutex);

    // Notify the UART task that a RingBuffer_t object has been enqueued.
    osSemaphoreRelease(*args.uartSemaphore);
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartLIS2MDL(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    // Run LIS2MDL setup routine. On failure, suspend this task.
    if(LIS2MDL_Setup(args.hspi) != LIS2MDL_SUCCESS) {
        report_error(args, "LIS2MDL SETUP ERROR");
        vTaskSuspend(NULL);
    }

    // Update magnetometer reading 8 times per second.
    LIS2MDL lis2mdl;
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read magnetometer from LIS2MDL sensor.
        if(LIS2MDL_Read(&lis2mdl, args.hspi) != LIS2MDL_SUCCESS) {
            report_error(args, "LIS2MDL READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest magnetometer reading to UART RingBuffer.
        send_update(args, lis2mdl);

        // Delay 125 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 125);
    }
}
