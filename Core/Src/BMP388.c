/*
 * BMP388.c
 *
 *  Created on: Jun 26, 2024
 *      Author: oglet
 */

#include "bmp388.h"
#include "cmsis_os.h"

/******************************/
/*		  		MACROS		   			*/
/******************************/

#define WRITE           (0x00)
#define READ            (0x80)
#define GPIOx						(GPIOB)
#define GPIO_PINx				(GPIO_PIN_10)
#define REG_WHO_AM_I		(0x00)
#define REG_CALIBRATION	(0x31)
#define REG_PWR_CTRL    (0x1B)
#define REG_OUTPUTS  		(0x04)
#define REG_PRES				(0x04)
#define REG_TEMP				(0x07)
#define PRES_ENABLE     (0x01)
#define TEMP_ENABLE     (0x02)
#define NORMAL_MODE     (0x30)

/******************************/
/*			HELPER FUNCTIONS			*/
/******************************/

static inline void compute_compensations(BMP388_COMP * const bmp388_comp,
																				 uint8_t const * const DATA) {
	bmp388_comp->T1 =  (float)((DATA[2] << 8) | DATA[1]) * 256.f;
	bmp388_comp->T2 =  (float)((DATA[4] << 8) | DATA[3]) / 1073741824.f;
	bmp388_comp->T3 =  (float)(DATA[5]) / 281474976710656.f;
	bmp388_comp->P1 = ((float)((DATA[7] << 8) | DATA[6]) - 16384.f) / 1048576.f;
	bmp388_comp->P2 = ((float)((DATA[9] << 8) | DATA[8]) - 16384.f) / 536870912.f;
	bmp388_comp->P3 =  (float)(DATA[10]) / 4294967296.f;
	bmp388_comp->P4 =  (float)(DATA[11]) / 137438953472.f;
	bmp388_comp->P5 =  (float)((DATA[13] << 8) | DATA[12]) * 8.f;
	bmp388_comp->P6 =  (float)((DATA[15] << 8) | DATA[14]) / 64.f;
	bmp388_comp->P7 =  (float)(DATA[16]) / 256.f;
	bmp388_comp->P8 =  (float)(DATA[17]) / 32768.f;
	bmp388_comp->P9 =  (float)((DATA[19] << 8) | DATA[18]) / 281474976710656.f;
	bmp388_comp->P10 = (float)(DATA[20]) / 281474976710656.f;
	bmp388_comp->P11 = (float)(DATA[21]) / 36893488147419103232.f;
}

static inline void compensate_temperature(BMP388 * const bmp388,
																					BMP388_COMP * const bmp388_comp,
																					float const uncomp_temp) {
	float partial_data1 = uncomp_temp - bmp388_comp->T1;
	bmp388->temperature = \
			(partial_data1 * bmp388_comp->T2) + \
			(partial_data1 * partial_data1) * bmp388_comp->T3;
}

static inline void compensate_pressure(BMP388 * const bmp388,
																			 BMP388_COMP * const bmp388_comp,
																			 float const uncomp_pres) {
	const float temperature = bmp388->temperature;
	float partial_out1 = \
	      (bmp388_comp->P5) + \
	      (bmp388_comp->P6 * temperature) + \
	      (bmp388_comp->P7 * temperature * temperature) + \
	      (bmp388_comp->P8 * temperature * temperature * temperature);
	float partial_out2 = uncomp_pres * \
			(bmp388_comp->P1 + \
			(bmp388_comp->P2 * temperature) + \
			(bmp388_comp->P3 * temperature * temperature) + \
			(bmp388_comp->P4 * temperature * temperature * temperature));
	float partial_out3 = (uncomp_pres * uncomp_pres) * \
			(bmp388_comp->P9 + bmp388_comp->P10 * temperature) + \
			(uncomp_pres * uncomp_pres * uncomp_pres * bmp388_comp->P11);\
	bmp388->pressure = partial_out1 + partial_out2 + partial_out3;
}

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

/******************************/
/*		  CORE FUNCTIONS		  	*/
/******************************/

BMP388_STATUS BMP388_Setup(BMP388 * const bmp388,
		 	 	 	 	 	 	 	 	 	 	 	 BMP388_COMP * const bmp388_comp,
													 SPI_HandleTypeDef * const hspi) {
	// If expected CHIP ID is not received, return failure
	const uint8_t TX1[] = { READ | REG_WHO_AM_I };
	uint8_t RX[22] = {0};

	transmit_receive(TX1, 1, RX, 2, hspi);

	if(RX[1] != 0x50) return BMP388_FAILURE;

	// Otherwise, configure sensor and receive calibration values
  const uint8_t TX2[] = {
      WRITE | REG_PWR_CTRL,
      WRITE | PRES_ENABLE | TEMP_ENABLE | NORMAL_MODE,
      READ  | REG_CALIBRATION
  };

  transmit_receive(TX2, 3, RX, 22, hspi);
  compute_compensations(bmp388_comp, RX);

  return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadTempPres(BMP388 * const bmp388,
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	BMP388_COMP * const bmp388_comp,
																	SPI_HandleTypeDef * const hspi) {
  const uint8_t TX[] = { READ  | REG_OUTPUTS };
  uint8_t RX[7] = {0};

  transmit_receive(TX, 1, RX, 7, hspi);

  float uncomp_temp = (float)((RX[6] << 16) | (RX[5] << 8) | RX[4]);
  float uncomp_pres = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

  compensate_temperature(bmp388, bmp388_comp, uncomp_temp);
  compensate_pressure(bmp388, bmp388_comp, uncomp_pres);

  return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadTemp(BMP388 * const bmp388,
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	BMP388_COMP * const bmp388_comp,
															SPI_HandleTypeDef * const hspi) {
	const uint8_t TX[] = { READ | REG_TEMP };
	uint8_t RX[4] = {0};

	transmit_receive(TX, 1, RX, 4, hspi);

	float uncomp_temp = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

	compensate_temperature(bmp388, bmp388_comp, uncomp_temp);

	return BMP388_SUCCESS;
}

BMP388_STATUS BMP388_ReadPres(BMP388 * const bmp388,
	 	 	 	 	 	 	 	 	 	 	 	 	 	  BMP388_COMP * const bmp388_comp,
															SPI_HandleTypeDef * const hspi) {
	const uint8_t TX[] = { READ | REG_PRES };
	uint8_t RX[4] = {0};

	transmit_receive(TX, 1, RX, 4, hspi);

	float uncomp_pres = (float)((RX[3] << 16) | (RX[2] << 8) | RX[1]);

	compensate_pressure(bmp388, bmp388_comp, uncomp_pres);

	return BMP388_SUCCESS;
}
