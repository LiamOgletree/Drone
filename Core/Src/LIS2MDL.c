/*
 * LIS2MDL.c
 *
 *  Created on: Sep 3, 2024
 *      Author: liamt
 */

#include "LIS2MDL.h"
#include "cmsis_os.h"
#include "math.h"

/******************************/
/*		  	 MACROS		      */
/******************************/

#define WRITE         	(0x00)
#define READ          	(0x80)
#define GPIOx			(GPIOB)
#define GPIO_PINx		(GPIO_PIN_4)
#define SET_DEFAULT		(0x00)
#define ENABLE_SENSOR	(0x80)
#define SET_SPI_4WIRE	(0x04)
#define REG_WHO_AM_I	(0x4F)
#define REG_CONFIGS		(0x60)
#define REG_OUTPUTS		(0x68)

/******************************/
/*		HELPER FUNCTIONS	  */
/******************************/

static inline void transmit_receive(uint8_t const * const TX,
									uint8_t const NUM_TX,
									uint8_t * const RX,
									uint8_t const NUM_RX,
									SPI_HandleTypeDef * const hspi) {
	taskENTER_CRITICAL();
	HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, (uint8_t *)TX, NUM_TX, HAL_MAX_DELAY);
	HAL_SPI_Receive(hspi, (uint8_t *)RX, NUM_RX, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_SET);
	taskEXIT_CRITICAL();
}

static inline void transmit(uint8_t const * const TX,
							uint8_t const NUM_TX,
							SPI_HandleTypeDef * const hspi) {
	taskENTER_CRITICAL();
	HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi, (uint8_t *)TX, NUM_TX, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOx, GPIO_PINx, GPIO_PIN_SET);
	taskEXIT_CRITICAL();
}

/******************************/
/*		 CORE FUNCTIONS		  */
/******************************/

LIS2MDL_STATUS LIS2MDL_Setup(LIS2MDL * const lis2mdl,
							 SPI_HandleTypeDef * const hspi) {
	// Configure sensor
	const uint8_t TX1[] = {
		WRITE | REG_CONFIGS,
		WRITE | ENABLE_SENSOR,
		WRITE | SET_DEFAULT,
		WRITE | SET_SPI_4WIRE
	};

	transmit(TX1, 4, hspi);

	// If expected WHO_AM_I value is not received, return failure
	const uint8_t TX2[] = { READ | REG_WHO_AM_I };
	uint8_t RX[1];

	transmit_receive(TX2, 1, RX, 1, hspi);

	if(RX[0] != 0x40) return LIS2MDL_FAILURE;

	return LIS2MDL_SUCCESS;
}

LIS2MDL_STATUS LIS2MDL_Read(LIS2MDL * const lis2mdl,
							SPI_HandleTypeDef * const hspi){
	const uint8_t TX[] = { READ | REG_OUTPUTS };
	uint8_t RX[6] = {0};

	transmit_receive(TX, 1, RX, 6, hspi);

	lis2mdl->X = (int16_t)(((uint16_t)RX[1] << 8) | RX[0]);
	lis2mdl->Y = (int16_t)(((uint16_t)RX[3] << 8) | RX[2]);
	lis2mdl->Z = (int16_t)(((uint16_t)RX[5] << 8) | RX[4]);

	return LIS2MDL_SUCCESS;
}
