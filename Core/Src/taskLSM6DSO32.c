/*
 * taskLSM6DSO32.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLSM6DSO32.h"
#include "sensor.h"
#include "LSM6DSO32.h"

// TODO:
//  1. Try to find some way to get rid of global variables, if it makes sense.
//  2. Try to recover LSM6DSO32 functionality instead of suspending task.

/******************************/
/*           MACROS           */
/******************************/

#define GYROSCOPE_DATA_TYPE     (LSM6DSO32_COMPENSATED)
#define ACCELEROMETER_DATA_TYPE (LSM6DSO32_COMPENSATED)

/******************************/
/*          GLOBALS           */
/******************************/

osThreadId_t RunGyroscopeHandle;
osThreadId_t RunAccelerometerHandle;

osThreadAttr_t const RunGyro_attributes = {
    .name = "RunGyro",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

osThreadAttr_t const RunAccel_attributes = {
    .name = "RunAccel",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline void report_error(const SENSOR_ARGS args,
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

static inline void send_update(const SENSOR_ARGS args,
                               const LSM6DSO32 lsm6dso32)
{
    // Create temporary RingBuffer_t object to place in UART RingBuffer.
    RingBufferUART_t const tmp = {.type = UPDATE_LSM6DSO32,
                                  .lsm6dso32 = lsm6dso32};

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

static void StartGyroscope(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_GYROSCOPE};

    // Update gyroscope reading 10 times per second.
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read gyroscope tilt from LSM6DSO32 sensor.
        if(LSM6DSO32_ReadGyro(&lsm6dso32,
                              args.hspi,
                              GYROSCOPE_DATA_TYPE)
                              != LSM6DSO32_SUCCESS) {
            report_error(args, "GYROSCOPE READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest gyroscope reading to UART RingBuffer.
        send_update(args, lsm6dso32);

        // Delay 100 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 100);
    }
}

static void StartAccelerometer(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_ACCELEROMETER};
    LSM6DSO32_OFFSET offset;

    // Take 100 readings from the accelerometer, compute the average
    //  on all axes, and then save these averages to be used as a
    //  software offset (see LSM6DSO32_UseOffset below).
    LSM6DSO32_CalibrateOffset(&lsm6dso32,
                              &offset,
                              args.hspi,
                              ACCELEROMETER_DATA_TYPE);

    // Update accelerometer reading 50 times per second.
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read accelerometer from LSM6DSO32 sensor.
        if(LSM6DSO32_ReadAccel(&lsm6dso32,
                               args.hspi,
                               ACCELEROMETER_DATA_TYPE)
                               != LSM6DSO32_SUCCESS) {
            report_error(args, "ACCELEROMETER READ ERROR");
            vTaskSuspend(NULL);
        }

        // Incorporate computed offset in x, y, and z directions.
        LSM6DSO32_UseOffset(&lsm6dso32, &offset);

        // Send latest accelerometer reading to UART RingBuffer.
        send_update(args, lsm6dso32);

        // Delay 100 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 20);
    }
}

void StartLSM6DSO32(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    // Run LSM6DSO32 setup routine before starting gyroscope or accelerometer
    //  tasks. On failure, suspend before starting subtasks.
    if(LSM6DSO32_Setup(args.hspi) != LSM6DSO32_SUCCESS) {
        report_error(args, "LSM6DSO32 SETUP ERROR");
        vTaskSuspend(NULL);
    }

    // Create and start gyroscope and acceleromter tasks.
    RunGyroscopeHandle     = osThreadNew(StartGyroscope,
                                         (void*) argument,
                                         &RunGyro_attributes);
    RunAccelerometerHandle = osThreadNew(StartAccelerometer,
                                         (void*) argument,
                                         &RunAccel_attributes);

    // Since this task is only used for setup and to start the gyroscope
    //  and accelerometer tasks, delete this task from the FreeRTOS kernel.
    vTaskDelete(NULL);
}
