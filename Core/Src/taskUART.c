/*
 * taskUART.c
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */


#include "taskUART.h"
#include "sensor.h"
#include <stdio.h>
#include <string.h>

// TODO:
//  1. Determine appropriate value for HAL_TIMEOUT instead of ~100 ms.
//  2. Instead of suspending task, try to recover UART functionality.
//  3. Try to find some way to make long sprintf statements with lots of
//     /n and /r characters readable.

/******************************/
/*      HELPER FUNCTIONS      */
/******************************/

static inline UART_STATUS transmit(UART_HandleTypeDef * const huart,
                                   char * const buf)
{
    if(HAL_UART_Transmit(huart,
                         (uint8_t*)buf,
                         strlen(buf),
                         (uint32_t)1600000) // ~100 ms
                         != HAL_OK) {
        return UART_FAILURE;
    }

    return UART_SUCCESS;
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartTaskUART(void *argument)
{
    SENSOR_ARGS const args = *(SENSOR_ARGS*)argument;

    char buf[210];
    sprintf(buf, "%c[0;0H\n"
                 "Temperature:\t\t\tC\r\n"
                 "Pressure:\t\t\tPa\r\n"
                 "Magnetometer:\r\n"
                 "\tX:\t\t\tmgauss\r\n"
                 "\tY:\t\t\tmgauss\r\n"
                 "\tZ:\t\t\tmgauss\r\n"
                 "\tHeading:\t\tdeg\r\n"
                 "Gyroscope:\r\n"
                 "\tX:\t\t\tmdps\r\n"
                 "\tY:\t\t\tmdps\r\n"
                 "\tZ:\t\t\tmdps\r\n"
                 "Accelerometer:\r\n"
                 "\tX:\t\t\tmg\r\n"
                 "\tY:\t\t\tmg\r\n"
                 "\tZ:\t\t\tmg\r\n",
                 0x1b);

    if(transmit(args.huart, buf) != UART_SUCCESS) {
        vTaskSuspend(NULL);
    }

    RingBuffer_t update;
    for(;;) {
        osSemaphoreAcquire(*args.uartSemaphore, osWaitForever);
        if(RingBuffer_dequeue(args.uart_rb, &update) != RB_SUCCESS) {
            sprintf(buf, "%c[0;0H RINGBUFFER DEQUEUE ERROR", 0x1b);
            transmit(args.huart, buf);
            vTaskSuspend(NULL);
        }

        switch(update.type) {
        case UPDATE_BMP388:
            if(update.bmp388.sensor == BMP388_TEMPERATURE) {
                sprintf(buf, "%c[0;0H"
                             "\n"
                             "\t\t%5.2f    ",
                             0x1b,
                             update.bmp388.temperature);
            } else {
                sprintf(buf, "%c[0;0H"
                             "\n\n"
                             "\t\t%5.2f    ",
                             0x1b,
                             update.bmp388.pressure);
            }
            break;
        case UPDATE_LIS2MDL:
            sprintf(buf, "%c[0;0H"
                         "\n\n\n\n"
                         "\t\t%hd    \r\n"
                         "\t\t%hd    \r\n"
                         "\t\t%hd    \r\n"
                         "\t\t%5.2f    ",
                         0x1b,
                         update.lis2mdl.X,
                         update.lis2mdl.Y,
                         update.lis2mdl.Z,
                         update.lis2mdl.heading);
            break;
        case UPDATE_LSM6DSO32:
            if(update.lsm6dso32.sensor == LSM6DSO32_GYROSCOPE) {
                sprintf(buf, "%c[0;0H"
                             "\n\n\n\n\n\n\n\n\n"
                             "\t\t%5.2f    \r\n"
                             "\t\t%5.2f    \r\n"
                             "\t\t%5.2f    ",
                             0x1b,
                             update.lsm6dso32.X,
                             update.lsm6dso32.Y,
                             update.lsm6dso32.Z);
            } else {
                sprintf(buf, "%c[0;0H"
                             "\n\n\n\n\n\n\n\n\n\n\n\n\n"
                             "\t\t%5.2f    \r\n"
                             "\t\t%5.2f    \r\n"
                             "\t\t%5.2f    ",
                             0x1b,
                             update.lsm6dso32.X,
                             update.lsm6dso32.Y,
                             update.lsm6dso32.Z);
            }
            break;
        case UPDATE_ERROR:
            sprintf(buf, "%c[0;0H"
                         "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                         "%s    ",
                         0x1b,
                         update.error_buf);
            break;
        default:
            sprintf(buf, "%c[0;0H"
                         "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                         "UART RECEIVED INVALID UPDATE TYPE   ",
                         0x1b);
            break;
        }

        if(transmit(args.huart, buf) != UART_SUCCESS) {
            vTaskSuspend(NULL);
        }
    }
}
