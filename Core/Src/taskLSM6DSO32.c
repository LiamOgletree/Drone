/*
 * taskLSM6DSO32.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLSM6DSO32.h"
#include "sensor.h"
#include "LSM6DSO32.h"

void StartTaskLSM6DSO32(void *argument) {

	SENSOR_ARGS args = *(SENSOR_ARGS*)argument;
	LSM6DSO32 lsm6dso32;

	if(LSM6DSO32_Setup(&lsm6dso32, args.hspi) != LSM6DSO32_SUCCESS) {
		// do nothing for now
	}

  for(;;) {
  	if(LSM6DSO32_Read(&lsm6dso32, args.hspi) != LSM6DSO32_SUCCESS) {
  		// do nothing for now
  	}

  	RingBuffer_enqueue(args.uart_rb,
											 (RingBuffer_t){.type = UPDATE_LSM6DSO32,
																			.lsm6dso32 = lsm6dso32});
  	osSemaphoreRelease(*args.uartSemaphore);

    osDelay(125);
  }
}
