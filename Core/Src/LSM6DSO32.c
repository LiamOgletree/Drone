/*
 * LSM6DSO32.c
 *
 *  Created on: Sep 6, 2024
 *      Author: liamt
 */

#include "LSM6DSO32.h"
#include "cmsis_os.h"

/******************************/
/*		  		MACROS		   			*/
/******************************/

#define WRITE         (0x00)
#define READ          (0x80)
#define GPIOx					(GPIOA)
#define GPIO_PINx			(GPIO_PIN_8)
#define REG_WHO_AM_I	(0x0F)
#define REG_CTRL1_XL	(0x10)
#define REG_CTRL2_G		(0x11)
#define REG_CTRL6_C		(0x15)
#define REG_CTRL7_G		(0x16)
#define SET_ODR_104HZ	(0x40)
#define EN_XL_OFFSET	(0x02)
#define SET_XL_WEIGHT	(0x08)
#define REG_OUTPUTS		(0x22)

/******************************/
/*			HELPER FUNCTIONS			*/
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
/*		  CORE FUNCTIONS		  	*/
/******************************/

LSM6DSO32_STATUS LSM6DSO32_Setup(LSM6DSO32 * const lsm6dso32,
																 SPI_HandleTypeDef * const hspi) {
	const uint8_t TX1[] = {
		WRITE | REG_CTRL1_XL,
		WRITE | SET_ODR_104HZ,
		WRITE | SET_ODR_104HZ
	};

	const uint8_t TX2[] = {
		WRITE | REG_CTRL6_C,
		WRITE | SET_XL_WEIGHT,
		WRITE | EN_XL_OFFSET
	};

	transmit(TX1, 3, hspi);
	transmit(TX2, 3, hspi);

	return LSM6DSO32_SUCCESS;
}

LSM6DSO32_STATUS LSM6DSO32_Read(LSM6DSO32 * const lsm6dso32,
																SPI_HandleTypeDef * const hspi) {
	const uint8_t TX[] = { READ  | REG_OUTPUTS };
	uint8_t RX[12] = {0};

	transmit_receive(TX, 1, RX, 12, hspi);

	lsm6dso32->G_X = (float)(((uint16_t)RX[1] << 8) | RX[0]) * 0.00007636f;
	lsm6dso32->G_Y = (float)(((uint16_t)RX[3] << 8) | RX[2]) * 0.00007636f;
	lsm6dso32->G_Z = (float)(((uint16_t)RX[5] << 8) | RX[4]) * 0.00007636f;

	lsm6dso32->A_X = (float)(((uint16_t)RX[7]  << 8) | RX[6])  * 0.00119641f;
	lsm6dso32->A_Y = (float)(((uint16_t)RX[9]  << 8) | RX[8])  * 0.00119641f;
	lsm6dso32->A_Z = (float)(((uint16_t)RX[11] << 8) | RX[10]) * 0.00119641f;

	return LSM6DSO32_SUCCESS;
}
