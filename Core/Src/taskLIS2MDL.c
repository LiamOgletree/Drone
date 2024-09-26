/*
 * taskLIS2MDL.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLIS2MDL.h"
#include "sensor.h"
#include "LIS2MDL.h"

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
                               LIS2MDL const lis2mdl)
{
    RingBuffer_t const tmp = {.type = UPDATE_LIS2MDL,
                              .lis2mdl = lis2mdl};
    RingBuffer_enqueue(args.uart_rb, tmp);
    osSemaphoreRelease(*args.uartSemaphore);
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartTaskLIS2MDL(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    if(LIS2MDL_Setup(args.hspi) != LIS2MDL_SUCCESS) {
        report_error(args, "LIS2MDL SETUP ERROR");
        vTaskSuspend(NULL);
    }

    LIS2MDL lis2mdl;
    for(;;) {
        if(LIS2MDL_Read(&lis2mdl, args.hspi) != LIS2MDL_SUCCESS) {
            report_error(args, "LIS2MDL READ ERROR");
            vTaskSuspend(NULL);
        }

        send_update(args, lis2mdl);
        osDelay(125);
    }
}
