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

osThreadId_t TaskTemperatureHandle;
osThreadId_t TaskPressureHandle;
BMP388_COMP bmp388_comp;

osThreadAttr_t const TaskTemp_attributes = {
    .name = "TaskTemp",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal1,
};

osThreadAttr_t const TaskPres_attributes = {
    .name = "TaskPres",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal1,
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
    RingBuffer_t const tmp = {.type = UPDATE_ERROR,
                              .error_buf = message};
    RingBuffer_enqueue(args.uart_rb, tmp);
    osSemaphoreRelease(*args.uartSemaphore);
}

static inline void send_update(SENSOR_ARGS const args,
                               BMP388 const bmp388)
{
    RingBuffer_t const tmp = {.type = UPDATE_BMP388,
                              .bmp388 = bmp388};
    RingBuffer_enqueue(args.uart_rb, tmp);
    osSemaphoreRelease(*args.uartSemaphore);
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

static void StartTaskTemperature(void *argument)
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
    for(;;) {
        if(BMP388_ReadTemp(&bmp388, &bmp388_comp, TEMPERATURE_DATA_TYPE,
                           args.hspi) != BMP388_SUCCESS) {
            report_error(args, "TEMPERATURE READ ERROR");
            vTaskSuspend(NULL);
        }

        send_update(args, bmp388);
        osDelay(1000);
    }
}

static void StartTaskPressure(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    BMP388 bmp388 = {.sensor = BMP388_PRESSURE};

    // Wait for temperature task to run once and provide a value for
    //  compensate_pressure function to work.
    osSemaphoreAcquire(bmp388SemaphoreHandle, osWaitForever);

    // Update pressure reading 40 times per second.
    for(;;) {
        if(BMP388_ReadPres(&bmp388,
                           &bmp388_comp,
                           PRESSURE_DATA_TYPE,
                           args.hspi)
                           != BMP388_SUCCESS) {
            report_error(args, "PRESURE READ ERROR");
            vTaskSuspend(NULL);
        }

        send_update(args, bmp388);
        osDelay(25);
    }
}

void StartTaskBMP388(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    if(BMP388_Setup(&bmp388_comp, args.hspi) != BMP388_SUCCESS) {
        report_error(args, "BMP388 SETUP ERROR");
        vTaskSuspend(NULL);
    }

    bmp388SemaphoreHandle = osSemaphoreNew(1, 0, &bmp388Semaphore_attributes);
    TaskTemperatureHandle = osThreadNew(StartTaskTemperature,
                                        (void*) argument,
                                        &TaskTemp_attributes);
    TaskPressureHandle    = osThreadNew(StartTaskPressure,
                                        (void*) argument,
                                        &TaskPres_attributes);
    vTaskSuspend(NULL);
}
