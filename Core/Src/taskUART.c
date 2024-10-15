/*
 * taskUART.c
 *
 *  Created on: Sep 9, 2024
 *      Author: liamt
 */

#include "taskUART.h"
#include "taskShared.h"
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
    UART_STATUS status = UART_SUCCESS;

    // Conduct UART transaction to send message.
    if(HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), HAL_TIMEOUT)
            != HAL_OK) {
        status = UART_FAILURE;
    }

    return status;
}

static inline UART_STATUS define_template(char * const buf)
{
    HAL_StatusTypeDef status = UART_SUCCESS;

    // Place UART terminal template in buffer.
    if(sprintf(buf, "%c[0;0H\n"
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
                 "\tZ:\t\t\tmg\r\n"
                 "State Machine:\r\n"
                 "\tpos_x:\t\t\tm\r\n"
                 "\tvel_x:\t\t\tm/s\r\n"
                 "\tacc_x:\t\t\tm/s^2\r\n"
                 "\tpos_y:\t\t\tm\r\n"
                 "\tvel_y:\t\t\tm/s\r\n"
                 "\tacc_y:\t\t\tm/s^2\r\n"
                 "\tpos_z:\t\t\tm\r\n"
                 "\tvel_z:\t\t\tm/s\r\n"
                 "\tacc_z:\t\t\tm/s^2\r\n",
                 0x1b)
            < 0) {
        status = UART_FAILURE;
    }

    return status;
}

static inline void set_pressure_reading(char * const buf,
                                        BMP388 const * const bmp388)
{
    sprintf(buf, "%c[0;0H"
                 "\n\n"
                 "\t\t%5.2f   \t\t\t%3.2f sec     ",
                 0x1b,
                 bmp388->pressure,
                 bmp388->timestamp / TIM5_CLK_SPEED);
}

static inline void set_temperature_reading(char * const buf,
                                           BMP388 const * const bmp388)
{
    sprintf(buf, "%c[0;0H"
                 "\n"
                 "\t\t%5.2f   \t\t\t%3.2f sec     ",
                 0x1b,
                 bmp388->temperature,
                 bmp388->timestamp / TIM5_CLK_SPEED);
}

static inline void set_magnetometer_reading(char * const buf,
                                            LIS2MDL const * const lis2mdl)
{
    float const timestamp = lis2mdl->timestamp / TIM5_CLK_SPEED;
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n"
                 "\t\t%hd    \t\t\t\t%3.2f sec      \r\n"
                 "\t\t%hd    \t\t\t\t%3.2f sec      \r\n"
                 "\t\t%hd    \t\t\t%3.2f sec      \r\n"
                 "\t\t%5.2f    ",
                 0x1b,
                 lis2mdl->X,
                 timestamp,
                 lis2mdl->Y,
                 timestamp,
                 lis2mdl->Z,
                 timestamp,
                 lis2mdl->heading);
}

static inline void set_gyroscope_reading(char * const buf,
                                         LSM6DSO32 const * const lsm6dso32)
{
    float const timestamp = lsm6dso32->timestamp / TIM5_CLK_SPEED;
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n\n\n\n\n\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     \r\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     \r\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     ",
                 0x1b,
                 lsm6dso32->X,
                 timestamp,
                 lsm6dso32->Y,
                 timestamp,
                 lsm6dso32->Z,
                 timestamp);
}

static inline void set_accelerometer_reading(char * const buf,
                                             LSM6DSO32 const * const lsm6dso32)
{
    float const timestamp = lsm6dso32->timestamp / TIM5_CLK_SPEED;
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n\n\n\n\n\n\n\n\n\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     \r\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     \r\n"
                 "\t\t%5.2f    \t\t\t%3.2f sec     ",
                 0x1b,
                 lsm6dso32->X,
                 timestamp,
                 lsm6dso32->Y,
                 timestamp,
                 lsm6dso32->Z,
                 timestamp);
}

static inline void set_sm_output(char * const buf,
                                 StateMachine_t const * const state)
{
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f\r\n"
                 "\t\t%5.2f",
                 0x1b,
                 state->pos_x,
                 state->vel_x,
                 state->acc_x,
                 state->pos_y,
                 state->vel_y,
                 state->acc_y,
                 state->pos_z,
                 state->vel_z,
                 state->acc_z);
}

static inline void set_rb_error_notification(char * const buf,
                                             char const * const error_buf)
{
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                 "%s    ",
                 0x1b,
                 error_buf);
}

static inline void set_type_error_notification(char * const buf,
                                               char const * const error_buf)
{
    sprintf(buf, "%c[0;0H"
                 "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n"
                 "UART RECEIVED INVALID RB TYPE   ",
                 0x1b);
}

/******************************/
/*       CORE FUNCTIONS       */
/******************************/

void StartUART(void *argument)
{
    TASK_ARGS const args = *(TASK_ARGS*)argument;

    char buf[360];

    // Set character buffer to hold UART template.
    if(define_template(buf) != UART_SUCCESS) {
        vTaskSuspend(NULL);
    }

    // Transmit template to terminal using UART.
    if(transmit(args.huart, buf) != UART_SUCCESS) {
        vTaskSuspend(NULL);
    }

    RingBuffer_t update;
    RB_STATUS status;
    // Run UART task indefinitely.
    for(;;) {
        // Wait for producer or state machine task to send an update
        //  to the UART ring buffer and post a semaphore.
        osSemaphoreAcquire(*args.uartSemaphore, osWaitForever);

        // Retrieve latest update from UART ring buffer with mutex
        //  to prevent race conditions possibly corrupting data.
        osMutexAcquire(*args.uartMutex, osWaitForever);
        status = RingBuffer_dequeue(args.uart_rb, &update);
        osMutexRelease(*args.uartMutex);

        // If unsuccessful retrieving an update, print error to the
        //  terminal and suspend UART task.
        if(status != RB_SUCCESS) {
            sprintf(buf, "%c[0;0H RINGBUFFER DEQUEUE ERROR", 0x1b);
            transmit(args.huart, buf);
            vTaskSuspend(NULL);
        }

        switch(update.type) {
        // If updating pressure reading on the terminal:
        case RB_PRESSURE:
            set_pressure_reading(buf, &update.bmp388);
            break;
        // If updating temperature reading on the terminal:
        case RB_TEMPERATURE:
            set_temperature_reading(buf, &update.bmp388);
            break;
        // If updating magnetometer reading on the terminal:
        case RB_MAGNETOMETER:
            set_magnetometer_reading(buf, &update.lis2mdl);
            break;
        // If updating gyroscope reading on the terminal:
        case RB_GYROSCOPE:
            set_gyroscope_reading(buf, &update.lsm6dso32);
            break;
        // If updating accelerometer reading on the terminal:
        case RB_ACCELEROMETER:
            set_accelerometer_reading(buf, &update.lsm6dso32);
            break;
        // If updating the state machine output on the terminal:
        case RB_STATEMACHINE:
            set_sm_output(buf, &update.state);
            break;
        // If notifying user of an error via the terminal:
        case RB_ERROR:
            set_rb_error_notification(buf, update.error_buf);
            break;
        // Otherwise, notify user of invalid type error:
        default:
            set_type_error_notification(buf, update.error_buf);
            break;
        }

        // Display the contents of buf on the terminal screen.
        if(transmit(args.huart, buf) != UART_SUCCESS) {
            vTaskSuspend(NULL);
        }
    }
}
