/*
 * taskBMP388.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskBMP388.h"
#include "sensor.h"
#include "BMP388.h"
#include <stdbool.h>

// TODO:
//  1. Try to find some way to get rid of global variables, if it makes sense.
//  2. Try to recover BMP388 functionality instead of suspending task.

/******************************/
/*           MACROS           */
/******************************/

#define TEMPERATURE_DATA_TYPE (BMP388_COMPENSATED)
#define PRESSURE_DATA_TYPE    (BMP388_COMPENSATED)

/******************************/
/*          GLOBALS           */
/******************************/

osThreadId_t RunTemperatureHandle;
osThreadId_t RunPressureHandle;
BMP388_COMP bmp388_comp;

osThreadAttr_t const RunTemp_attributes = {
    .name = "RunTemp",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

osThreadAttr_t const RunPres_attributes = {
    .name = "RunPres",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

osSemaphoreId_t bmp388SemaphoreHandle;
const osSemaphoreAttr_t bmp388Semaphore_attributes = {
  .name = "bmp388Semaphore"
};

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
                               BMP388 const bmp388)
{
    // Create temporary RingBuffer_t object to place in UART RingBuffer.
    RingBufferUART_t const tmp = {.type = UPDATE_BMP388,
                                  .bmp388 = bmp388};

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

static void StartTemperature(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    BMP388 bmp388 = {.sensor = BMP388_TEMPERATURE};

    // Initial temperature read to update global temperature variable
    //  in bmp388.c for pressure compensations.
    if(BMP388_ReadTemp(&bmp388, &bmp388_comp, TEMPERATURE_DATA_TYPE,
                       args.hspi) != BMP388_SUCCESS) {
        report_error(args, "TEMPERATURE READ ERROR");
        vTaskSuspend(NULL);
    }

    // Let pressure task know it can start, because initial temperature
    //  is received and available for pressure compensation.
    osSemaphoreRelease(bmp388SemaphoreHandle);

    // Update temperature reading once per second.
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read temperature from BMP388 sensor.
        if(BMP388_ReadTemp(&bmp388, &bmp388_comp, TEMPERATURE_DATA_TYPE,
                           args.hspi) != BMP388_SUCCESS) {
            report_error(args, "TEMPERATURE READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest temperature reading to UART RingBuffer.
        send_update(args, bmp388);

        // Delay 1 second from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

static void StartPressure(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    BMP388 bmp388 = {.sensor = BMP388_PRESSURE};

    // Wait for temperature task to run once and provide a value for
    //  compensate_pressure function to work.
    osSemaphoreAcquire(bmp388SemaphoreHandle, osWaitForever);

    // Update pressure reading 25 times per second.
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read pressure from BMP388 sensor.
        if(BMP388_ReadPres(&bmp388,
                           &bmp388_comp,
                           PRESSURE_DATA_TYPE,
                           args.hspi)
                           != BMP388_SUCCESS) {
            report_error(args, "PRESURE READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest pressure reading to UART RingBuffer.
        send_update(args, bmp388);

        // Delay 40 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 40);
    }
}

void StartBMP388(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    // Run BMP388 setup routine before starting pressure or temperature
    //  tasks. On failure, suspend before starting subtasks.
    if(BMP388_Setup(&bmp388_comp, args.hspi) != BMP388_SUCCESS) {
        report_error(args, "BMP388 SETUP ERROR");
        vTaskSuspend(NULL);
    }

    // Create the private BMP388 semaphore which will allow the temperature
    //  task to notify the pressure task that it can start.
    bmp388SemaphoreHandle = osSemaphoreNew(1, 0, &bmp388Semaphore_attributes);

    // Create and start temperature and pressure tasks.
    RunTemperatureHandle = osThreadNew(StartTemperature,
                                       (void*) argument,
                                       &RunTemp_attributes);
    RunPressureHandle    = osThreadNew(StartPressure,
                                       (void*) argument,
                                       &RunPres_attributes);

    // Since this task is only used for setup and to start the temperature
    //  and pressure tasks, delete this task from the FreeRTOS kernel.
    vTaskDelete(NULL);
}
