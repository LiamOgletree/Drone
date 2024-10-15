/*
 * sensorShared.h
 *
 *  Created on: Oct 11, 2024
 *      Author: liamt
 */

#ifndef INC_SENSORSHARED_H_
#define INC_SENSORSHARED_H_

#include "main.h"
#include "cmsis_os.h"

typedef enum {
    SENSOR_SUCCESS,
    SENSOR_FAILURE
} SENSOR_STATUS;

SENSOR_STATUS sensor_tx_rx(uint8_t const * const TX,
                           uint8_t const NUM_TX,
                           uint8_t * const RX,
                           uint8_t const NUM_RX,
                           SPI_HandleTypeDef * const hspi,
                           GPIO_TypeDef * const GPIOx,
                           uint16_t const GPIO_PINx);
SENSOR_STATUS sensor_tx(uint8_t const * const TX,
                        uint8_t const NUM_TX,
                        SPI_HandleTypeDef * const hspi,
                        GPIO_TypeDef * const GPIOx,
                        uint16_t const GPIO_PINx);

#endif /* INC_SENSORSHARED_H_ */
