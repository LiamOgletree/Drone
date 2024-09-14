/*
 * taskBMP388.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskBMP388.h"
#include "sensor.h"
#include "BMP388.h"

void StartTaskBMP388(void *argument){

    SENSOR_ARGS args = *(SENSOR_ARGS*)argument;
    BMP388 bmp388;

    if(BMP388_Setup(&bmp388, args.hspi) != BMP388_SUCCESS) {
        // do nothing for now
    }

    for(;;) {
        if(BMP388_ReadTempPres(&bmp388, args.hspi) != BMP388_SUCCESS) {
            // do nothing for now
        }

        RingBuffer_enqueue(args.uart_rb,
                           (RingBuffer_t){.type = UPDATE_BMP388,
                                          .bmp388 = bmp388});
        osSemaphoreRelease(*args.uartSemaphore);

        osDelay(50);
    }
}
