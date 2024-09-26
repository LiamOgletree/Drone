/*
 * taskLSM6DSO32.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLSM6DSO32.h"
#include "sensor.h"
#include "LSM6DSO32.h"

/******************************/
/*           MACROS           */
/******************************/

#define GYROSCOPE_DATA_TYPE     (LSM6DSO32_COMPENSATED)
#define ACCELEROMETER_DATA_TYPE (LSM6DSO32_COMPENSATED)

/******************************/
/*          GLOBALS           */
/******************************/

osThreadId_t TaskGyroscopeHandle;
osThreadId_t TaskAccelerometerHandle;

osThreadAttr_t const TaskGyro_attributes = {
    .name = "TaskGyro",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal1,
};

osThreadAttr_t const TaskAccel_attributes = {
    .name = "TaskAccel",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t) osPriorityNormal1,
};

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline void report_error(const SENSOR_ARGS args,
                                char * const message)
{
    RingBuffer_t const tmp = {.type = UPDATE_ERROR,
                              .error_buf = message};
    RingBuffer_enqueue(args.uart_rb, tmp);
    osSemaphoreRelease(*args.uartSemaphore);
}

static inline void send_update(const SENSOR_ARGS args,
                               const LSM6DSO32 lsm6dso32)
{
    RingBuffer_t const tmp = {.type = UPDATE_LSM6DSO32,
                              .lsm6dso32 = lsm6dso32};
    RingBuffer_enqueue(args.uart_rb, tmp);
    osSemaphoreRelease(*args.uartSemaphore);
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

static void StartTaskGyroscope(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_GYROSCOPE};

    for(;;) {
        if(LSM6DSO32_ReadGyro(&lsm6dso32,
                              args.hspi,
                              GYROSCOPE_DATA_TYPE)
                              != LSM6DSO32_SUCCESS) {
            report_error(args, "GYROSCOPE READ ERROR");
            vTaskSuspend(NULL);
        }

        send_update(args, lsm6dso32);
        osDelay(100);
    }
}

static void StartTaskAccelerometer(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;
    LSM6DSO32 lsm6dso32 = {.sensor = LSM6DSO32_ACCELEROMETER};
    LSM6DSO32_OFFSET offset;

    LSM6DSO32_CalibrateOffset(&lsm6dso32,
                              &offset,
                              args.hspi,
                              ACCELEROMETER_DATA_TYPE);

    for(;;) {
        if(LSM6DSO32_ReadAccel(&lsm6dso32,
                               args.hspi,
                               ACCELEROMETER_DATA_TYPE)
                               != LSM6DSO32_SUCCESS) {
            report_error(args, "ACCELEROMETER READ ERROR");
            vTaskSuspend(NULL);
        }

        LSM6DSO32_UseOffset(&lsm6dso32, &offset);

        send_update(args, lsm6dso32);
        osDelay(20);
    }
}

void StartTaskLSM6DSO32(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    if(LSM6DSO32_Setup(args.hspi) != LSM6DSO32_SUCCESS) {
        report_error(args, "LSM6DSO32 SETUP ERROR");
        vTaskSuspend(NULL);
    }

    TaskGyroscopeHandle     = osThreadNew(StartTaskGyroscope,
                                          (void*) argument,
                                          &TaskGyro_attributes);
    TaskAccelerometerHandle = osThreadNew(StartTaskAccelerometer,
                                          (void*) argument,
                                          &TaskAccel_attributes);
    vTaskSuspend(NULL);
}
