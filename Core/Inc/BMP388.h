/*
 * BMP388.h
 *
 *  Created on: Jun 26, 2024
 *      Author: oglet
 */

#ifndef SRC_BMP388_H_
#define SRC_BMP388_H_

#include "main.h"

typedef enum {
    BMP388_TEMPERATURE,
    BMP388_PRESSURE
} BMP388_SENSOR;

typedef struct BMP388 {
    BMP388_SENSOR sensor;
    uint32_t timestamp;
    union {
        float temperature;
        float pressure;
    };
} BMP388;

typedef struct BMP388_COMP {
    float T1, T2, T3,
          P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
} BMP388_COMP;

typedef enum {
    BMP388_SUCCESS,
    BMP388_FAILURE,
    BMP388_SPI_TX_FAILURE,
    BMP388_SPI_RX_FAILURE,
    BMP388_WHOAMI_FAILURE
} BMP388_STATUS;

typedef enum {
    BMP388_RAW,
    BMP388_COMPENSATED
} BMP388_DATA_TYPE;

BMP388_STATUS BMP388_Setup(BMP388_COMP * const bmp388_comp,
                           SPI_HandleTypeDef * const hspi);
BMP388_STATUS BMP388_ReadTemp(BMP388 * const bmp388,
                              BMP388_COMP const * const bmp388_comp,
                              BMP388_DATA_TYPE const data_type,
                              SPI_HandleTypeDef * const hspi);
BMP388_STATUS BMP388_ReadPres(BMP388 * const bmp388,
                              BMP388_COMP const * const bmp388_comp,
                              BMP388_DATA_TYPE const data_type,
                              SPI_HandleTypeDef * const hspi);

#endif /* SRC_BMP388_H_ */
