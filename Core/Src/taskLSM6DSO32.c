/*
 * taskLSM6DSO32.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include <sensorLSM6DSO32.h>
#include <taskShared.h>
#include "taskLSM6DSO32.h"

// TODO:
//  1. Try to find some way to get rid of global variables, if it makes sense.
//  2. Try to recover LSM6DSO32 functionality instead of suspending task.

/******************************/
/*           MACROS           */
/******************************/

#define G_DATA_TYPE (LSM6DSO32_COMPENSATED) // Gyroscope data type
#define A_DATA_TYPE (LSM6DSO32_COMPENSATED) // Accelerometer data type

/******************************/
/*          GLOBALS           */
/******************************/

osThreadId_t RunGyroscopeHandle;
osThreadId_t RunAccelerometerHandle;

osThreadAttr_t const RunGyro_attributes = {
    .name = "RunGyro",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

osThreadAttr_t const RunAccel_attributes = {
    .name = "RunAccel",
    .stack_size = 144 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

static void StartGyroscope(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    // Update gyroscope reading 10 times per second.
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_GYROSCOPE};
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read gyroscope tilt from LSM6DSO32 sensor.
        if(LSM6DSO32_ReadGyro(&lsm6dso32, args.hspi, G_DATA_TYPE)
                != LSM6DSO32_SUCCESS) {
            task_report_error(args, "GYROSCOPE READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest gyroscope reading to UART RingBuffer.
        RingBuffer_t const update = {.type = RB_GYROSCOPE, .lsm6dso32 = lsm6dso32};
        task_send_update_all(args, update);

        // Delay 100 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 100);
    }
}

static void StartAccelerometer(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_ACCELEROMETER};
    LSM6DSO32_OFFSET offset;

    // Take 100 readings from the accelerometer, compute the average
    //  on all axes, and then save these averages to be used as a
    //  software offset (see LSM6DSO32_UseOffset below).
    LSM6DSO32_CalibrateOffset(&lsm6dso32, &offset, args.hspi, A_DATA_TYPE);

    // Update accelerometer reading 25 times per second.
    TickType_t xLastWakeTime;
    for(;;) {
        // Read current tick time for vTaskDelayUntil() to make sure our
        //  delay is precise.
        xLastWakeTime = xTaskGetTickCount();

        // Read accelerometer from LSM6DSO32 sensor.
        if(LSM6DSO32_ReadAccel(&lsm6dso32, args.hspi, A_DATA_TYPE)
                != LSM6DSO32_SUCCESS) {
            task_report_error(args, "ACCELEROMETER READ ERROR");
            vTaskSuspend(NULL);
        }

        // Incorporate computed offset in x, y, and z directions.
        LSM6DSO32_UseOffset(&lsm6dso32, &offset);

        // Send latest accelerometer reading to UART RingBuffer.
        RingBuffer_t const update = {.type = RB_ACCELEROMETER, .lsm6dso32 = lsm6dso32};
        task_send_update_all(args, update);

        // Delay 100 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 40);
    }
}

void StartLSM6DSO32(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    // Run LSM6DSO32 setup routine before starting gyroscope or accelerometer
    //  tasks. On failure, suspend before starting subtasks.
    if(LSM6DSO32_Setup(args.hspi) != LSM6DSO32_SUCCESS) {
        task_report_error(args, "LSM6DSO32 SETUP ERROR");
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
