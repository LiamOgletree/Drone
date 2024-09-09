/*
 * taskLIS2MDL.c
 *
 *  Created on: Sep 8, 2024
 *      Author: liamt
 */

#include "taskLIS2MDL.h"
#include "sensor.h"
#include "LIS2MDL.h"

void StartTaskLIS2MDL(void *argument) {

	SENSOR_ARGS args = *(SENSOR_ARGS*)argument;
	LIS2MDL lis2mdl;

	if(LIS2MDL_Setup(&lis2mdl, args.hspi) != LIS2MDL_SUCCESS) {
		// do nothing for now
	}

  for(;;) {
  	if(LIS2MDL_Read(&lis2mdl, args.hspi) != LIS2MDL_SUCCESS) {
  		// do nothing for now
  	}

  	RingBuffer_enqueue(args.uart_rb,
  										 (RingBuffer_t){.type = UPDATE_LIS2MDL,
  																		.lis2mdl = lis2mdl});
  	osSemaphoreRelease(*args.uartSemaphore);

    osDelay(125);
  }
}
