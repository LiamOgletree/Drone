/*
 * taskLIS2MDL.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include <sensorLIS2MDL.h>
#include <taskShared.h>
#include "taskLIS2MDL.h"

// TODO:
//  1. Try to recover LIS2MDL functionality instead of suspending task.

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartLIS2MDL(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    // Run LIS2MDL setup routine. On failure, suspend this task.
    if(LIS2MDL_Setup(args.hspi) != LIS2MDL_SUCCESS) {
        task_report_error(args, "LIS2MDL SETUP ERROR");
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
            task_report_error(args, "LIS2MDL READ ERROR");
            vTaskSuspend(NULL);
        }

        // Send latest magnetometer reading to UART RingBuffer.
        RingBuffer_t const update = {.type = RB_MAGNETOMETER, .lis2mdl = lis2mdl};
        task_send_update_all(args, update);

        // Delay 125 milliseconds from the start of the last reading.
        vTaskDelayUntil(&xLastWakeTime, 125);
    }
}
