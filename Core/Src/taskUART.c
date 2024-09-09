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

void StartTaskUART(void *argument) {

	SENSOR_ARGS args = *(SENSOR_ARGS*)argument;

	char buf[124];
	sprintf(buf, "%c[0;0H\n"
							 "Temperature:\r\n"
							 "Pressure:\r\n"
							 "Magnetometer:\r\n"
							 "\tX:\r\n"
							 "\tY:\r\n"
	             "\tZ:\r\n"
							 "Gyroscope:\r\n"
							 "\tX:\r\n"
							 "\tY:\r\n"
							 "\tZ:\r\n"
							 "Accelerometer:\r\n"
							 "\tX:\r\n"
							 "\tY:\r\n"
							 "\tZ:\r\n",
							 0x1b);
	HAL_UART_Transmit(args.huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	RingBuffer_t update;
  for(;;) {
  	osSemaphoreAcquire(*args.uartSemaphore, osWaitForever);
  	RingBuffer_dequeue(args.uart_rb, &update);

  	switch(update.type) {
  		case UPDATE_BMP388:
  			sprintf(buf, "%c[0;0H"
  									 "\n"
  									 "\t\t%5.2f C\r\n"
  									 "\t\t%5.2f Pa",
										 0x1b,
										 update.bmp388.temperature,
										 update.bmp388.pressure);
				break;
  		case UPDATE_LIS2MDL:
  			sprintf(buf, "%c[0;0H"
  									 "\n\n\n\n"
										 "\t\t%hd\r\n"
										 "\t\t%hd\r\n"
										 "\t\t%hd",
										 0x1b,
										 update.lis2mdl.X,
										 update.lis2mdl.Y,
										 update.lis2mdl.Z);
				break;
  		case UPDATE_LSM6DSO32:
  			sprintf(buf, "%c[0;0H"
  									 "\n\n\n\n\n\n\n\n"
										 "\t\t%5.2f\r\n"
										 "\t\t%5.2f\r\n"
										 "\t\t%5.2f\r\n\n"
										 "\t\t%5.2f\r\n"
										 "\t\t%5.2f\r\n"
										 "\t\t%5.2f",
										 0x1b,
										 update.lsm6dso32.G_X,
										 update.lsm6dso32.G_Y,
										 update.lsm6dso32.G_Z,
										 update.lsm6dso32.A_X,
										 update.lsm6dso32.A_Y,
										 update.lsm6dso32.A_Z);
				break;
  	}

		HAL_UART_Transmit(args.huart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
  }
}
